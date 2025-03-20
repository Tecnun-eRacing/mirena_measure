#ifndef ekf_scheduler_hpp
#define ekf_scheduler_hpp

#include <rclcpp/clock.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "mirena_common/msg/wheel_speeds.hpp"
#include "mirena_common/msg/car.hpp"

#include "DBMMK1/dbmmk1.hpp"

class EKFScheduler
{
private:
    const rclcpp::Clock::SharedPtr _clock;
    rclcpp::Time _last_gps_update, _last_imu_update, _last_wss_update;
    DBMMK1::U _last_control_processed;
    DBMMK1::Z _last_measure_processed;

    mirena::ExtendedKalmanFilter<DBMMK1::STATE_DIM, DBMMK1::CONTROL_DIM, DBMMK1::MEASURE_DIM> _ekf;

public:
    EKFScheduler() = delete; /* A rclcpp::CLock must be supplied */
    EKFScheduler(const rclcpp::Clock::SharedPtr clock,
                 const DBMMK1::Parameters &model_parameters) : _clock(clock),
                                                               _ekf(DBMMK1::Model::build_ekf(model_parameters)) {}

    // DE DONDE COÑO SACO LA \a Y LA \delta (imagino que de un CarControls.msg????)

    void get_max_sensor_slack();

    void receive_gps(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    void receive_imu(const sensor_msgs::msg::Imu::SharedPtr msg);

    void receive_wss(const mirena_common::msg::WheelSpeeds::SharedPtr msg);

    mirena_common::msg::Car predict_state();
};

#endif