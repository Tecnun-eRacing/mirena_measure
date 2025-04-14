#ifndef ekf_scheduler_hpp
#define ekf_scheduler_hpp

#include <rclcpp/node.hpp>
#include <rclcpp/clock.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "mirena_common/msg/wheel_speeds.hpp"
#include "mirena_common/msg/car_control.hpp"
#include "mirena_common/msg/car.hpp"

#include "DBMMK1/dbmmk1.hpp"

#include <optional>

#define FIXED_FRAME "world"

// Using ENU convention (without U lmaooo)
struct CartesianCoords {
    double x;
    double y;
};

class GPSPos{
    public:
    GPSPos(double latitude, double longitude): _latitude(latitude), _longitude(longitude){}

    CartesianCoords to_cartesian(GPSPos& origin);

    private:
    double _latitude;
    double _longitude;
};

class EKFScheduler
{
private:
    const rclcpp::Node &_owner;
    rclcpp::Clock::ConstSharedPtr _clock;
    DBMMK1::U _last_control_processed;
    DBMMK1::Z _last_measure_processed;
    rclcpp::Time _last_ekf_update;
    rclcpp::Time _last_gps_update, _last_imu_update, _last_wss_update;

    std::optional<GPSPos> gps_origin;
    mirena::ExtendedKalmanFilter<DBMMK1::STATE_DIM, DBMMK1::CONTROL_DIM, DBMMK1::MEASURE_DIM> _ekf;

    DBMMK1::U get_control(double delta_t);
    void update_ekf(const DBMMK1::U& control, const DBMMK1::Z& measures);


    public:
    EKFScheduler() = delete; /* A rclcpp::CLock must be supplied */
    EKFScheduler(const rclcpp::Node &owner,
                 const DBMMK1::Parameters &model_parameters) : _owner(owner),
                                                               _clock(this->_owner.get_clock()),
                                                               _last_control_processed(DBMMK1::U::Zero()),
                                                               _last_measure_processed(DBMMK1::Z::Zero()),
                                                               _last_ekf_update(this->_clock->now()),
                                                               _last_gps_update(this->_last_ekf_update),
                                                               _last_imu_update(this->_last_ekf_update),
                                                               _last_wss_update(this->_last_ekf_update),
                                                               _ekf(DBMMK1::Model::build_ekf(model_parameters)) {}

    int64_t get_max_sensor_slack();

    void receive_gps(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    void receive_imu(const sensor_msgs::msg::Imu::SharedPtr msg);

    void receive_wss(const mirena_common::msg::WheelSpeeds::SharedPtr msg);

    void receive_control(const mirena_common::msg::CarControl::SharedPtr msg);

    mirena_common::msg::Car predict_state();

};

#endif