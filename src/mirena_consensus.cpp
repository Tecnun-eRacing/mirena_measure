#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "mirena_common/msg/wheel_speeds.hpp"
#include "mirena_common/msg/car_control.hpp"
#include "mirena_common/msg/car.hpp"
#include "mirena_common/const/mirena_const.h"

#include "ekf_scheduler.hpp"

#define GPS_SUB_TOPIC "sensors/gps"
#define IMU_SUB_TOPIC "sensors/imu"
#define WSS_SUB_TOPIC "sensors/wss"
#define CONTROL_SUB_TOPIC "control/car_control"
#define CAR_PUB_TOPIC "consensus/car_state"

#define CAR_FRAME_NAME "MirenaCar"

#define NODE_PUBLISH_PERIOD 100ms

using namespace std::chrono_literals;

class MirenaConsensusNode : public rclcpp::Node
{    
public:
    static constexpr DBMMK1::Parameters model_parameters = {
        MR_CONST_L_F,   // Distance from center of mass to front axis
        MR_CONST_K_F, // Front axis equivalent sideslip stiffness
        MR_CONST_L_R,   // Distance from center of mass to rear axis
        MR_CONST_K_R, // Rear axis equivalent sideslip stiffness
        MR_CONST_I_Z,  // Yaw inertia of vehicle body
        MR_CONST_MASS   // Mass of the vehicle
    };

    MirenaConsensusNode() : Node("mirena_consensus"),
                            _ekf_scheduler(*this, MirenaConsensusNode::model_parameters),
                            _gps_sub(this->create_subscription<sensor_msgs::msg::NavSatFix>(GPS_SUB_TOPIC, 10, std::bind(&MirenaConsensusNode::gps_callback, this, std::placeholders::_1))),
                            _imu_sub(this->create_subscription<sensor_msgs::msg::Imu>(IMU_SUB_TOPIC, 10, std::bind(&MirenaConsensusNode::imu_callback, this, std::placeholders::_1))),
                            _wss_sub(this->create_subscription<mirena_common::msg::WheelSpeeds>(WSS_SUB_TOPIC, 10, std::bind(&MirenaConsensusNode::wss_callback, this, std::placeholders::_1))),
                            _control_sub(this->create_subscription<mirena_common::msg::CarControl>(CONTROL_SUB_TOPIC, 10, std::bind(&MirenaConsensusNode::control_callback, this, std::placeholders::_1))),
                            _car_pub(this->create_publisher<mirena_common::msg::Car>(CAR_PUB_TOPIC, 10)),
                            _node_timer(this->create_timer(NODE_PUBLISH_PERIOD, std::bind(&MirenaConsensusNode::timer_callback, this)))
    {
        RCLCPP_INFO(this->get_logger(), "Sensor Consensus Node initialized.");
    }


private:
    // etc etc etc
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) { this->_ekf_scheduler.receive_gps(msg); }
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) { this->_ekf_scheduler.receive_imu(msg); }
    void wss_callback(const mirena_common::msg::WheelSpeeds::SharedPtr msg) { this->_ekf_scheduler.receive_wss(msg); }
    void control_callback(const mirena_common::msg::CarControl::SharedPtr msg) { this->_ekf_scheduler.receive_control(msg); }

    void timer_callback(){
        mirena_common::msg::Car car_estimate = this->_ekf_scheduler.predict_state();
        // Publish Car Estimate
        this->_car_pub->publish(car_estimate);

        // Update Car Frame

    };
    
    EKFScheduler _ekf_scheduler;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr _gps_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_sub;
    rclcpp::Subscription<mirena_common::msg::WheelSpeeds>::SharedPtr _wss_sub;
    rclcpp::Subscription<mirena_common::msg::CarControl>::SharedPtr _control_sub;
    rclcpp::Publisher<mirena_common::msg::Car>::SharedPtr _car_pub;

    rclcpp::TimerBase::SharedPtr _node_timer;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MirenaConsensusNode>());
    rclcpp::shutdown();
    return 0;
}
