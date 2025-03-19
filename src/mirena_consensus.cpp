#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include "mirena_common/msg/wheel_speeds.hpp"
#include "mirena_common/msg/car.hpp"

#define GPS_SUB_TOPIC "sensors/gps"
#define WSS_SUB_TOPIC "sensors/wss"
#define IMU_SUB_TOPIC "sensors/imu"
#define CAR_PUB_TOPIC "consensus/car"

class MirenaConsensusNode : public rclcpp::Node
{
public:
    MirenaConsensusNode() : Node("mirena_lidar"),
                            _gps_sub(this->create_subscription<sensor_msgs::msg::NavSatFix>(GPS_SUB_TOPIC, 10, std::bind(&MirenaConsensusNode::gps_callback, this, std::placeholders::_1))),
                            _imu_sub(this->create_subscription<sensor_msgs::msg::Imu>(IMU_SUB_TOPIC, 10, std::bind(&MirenaConsensusNode::imu_callback, this, std::placeholders::_1))),
                            _wss_sub(this->create_subscription<mirena_common::msg::WheelSpeeds>(WSS_SUB_TOPIC, 10, std::bind(&MirenaConsensusNode::wss_callback, this, std::placeholders::_1))),
                            _car_pub(this->create_publisher<mirena_common::msg::Car>(CAR_PUB_TOPIC, 10))
    {
        RCLCPP_INFO(this->get_logger(), "Sensor Consensus Node initialized.");
    }

private:
    // etc etc etc
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) const
    {
        // TODO
    }

    void imu_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) const
    {
        // TODO
    }

    void wss_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) const
    {
        // TODO
    }

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr _gps_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_sub;
    rclcpp::Subscription<mirena_common::msg::WheelSpeeds>::SharedPtr _wss_sub;
    rclcpp::Publisher<mirena_common::msg::Car>::SharedPtr _car_pub;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MirenaConsensusNode>());
    rclcpp::shutdown();
    return 0;
}
