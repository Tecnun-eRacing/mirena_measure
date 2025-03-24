#include "ekf_scheduler.hpp"

#include "rclcpp/logging.hpp"

int64_t EKFScheduler::get_max_sensor_slack()
{
    return std::max(this->_last_gps_update.nanoseconds(), this->_last_imu_update.nanoseconds(), this->_last_wss_update.nanoseconds());
}

void EKFScheduler::receive_gps(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    // Ensure order of messages (i think this can also be done with qos)
    if (this->_last_gps_update >= msg->header.stamp)
    {
        RCLCPP_INFO(this->_owner.get_logger(), "Dropping out-of-order gps message");
        return;
    }
    double delta_t = (msg->header.stamp.sec - this->_last_gps_update.seconds()) +
                     (msg->header.stamp.nanosec - (this->_last_gps_update.nanoseconds() % 1000000000)) / 1e9;
    this->_last_gps_update = msg->header.stamp;

    // TODO: UNPACK THE MESSAGE
    static_assert(false, "GPS UNPACKING NOT IMPLEMENTED");
    double p_x = 0;
    double p_y = 0;
    // Once unpacked

    // We predict the measures and overwrite the ones we know
    DBMMK1::U control = this->get_control(delta_t);
    DBMMK1::Z inferred_measure = this->_ekf.predict_measures(control);
    inferred_measure(0) = p_x;
    inferred_measure(1) = p_y;

    this->_ekf.update(control, inferred_measure);
}

void EKFScheduler::receive_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    // Ensure order of messages (i think this can also be done with qos)
    if (this->_last_imu_update >= msg->header.stamp)
    {
        RCLCPP_INFO(this->_owner.get_logger(), "Dropping out-of-order imu message");
        return;
    }
    double delta_t = (msg->header.stamp.sec - this->_last_imu_update.seconds()) +
                     (msg->header.stamp.nanosec - (this->_last_imu_update.nanoseconds() % 1000000000)) / 1e9;
    this->_last_imu_update = msg->header.stamp;

    // TODO: UNPACK THE MESSAGE
    static_assert(false, "IMU UNPACKING NOT IMPLEMENTED");
    double phi = 0;
    double omega = 0;
    // Once unpacked

    // We predict the measures and overwrite the ones we know
    DBMMK1::U control = this->get_control(delta_t);
    DBMMK1::Z inferred_measure = this->_ekf.predict_measures(control);
    inferred_measure(2) = phi;
    inferred_measure(3) = omega;

    this->_ekf.update(control, inferred_measure);
}

void EKFScheduler::receive_wss(const mirena_common::msg::WheelSpeeds::SharedPtr msg)
{
    // Ensure order of messages (i think this can also be done with qos)
    if (this->_last_wss_update >= msg->header.stamp)
    {
        RCLCPP_INFO(this->_owner.get_logger(), "Dropping out-of-order wss message");
        return;
    }
    double delta_t = (msg->header.stamp.sec - this->_last_wss_update.seconds()) +
                     (msg->header.stamp.nanosec - (this->_last_wss_update.nanoseconds() % 1000000000)) / 1e9;
    this->_last_wss_update = msg->header.stamp;

    // DO NOTHING FOR NOW
}

void EKFScheduler::receive_control(const mirena_common::msg::AckermannDriveStamped::SharedPtr msg)
{
    this->_last_control_processed(0) = msg->drive.acceleration;
    this->_last_control_processed(1) = msg->drive.steering_angle;
}

mirena_common::msg::Car EKFScheduler::predict_state()
{
    return mirena_common::msg::Car();
}

// Returns the last control value used but with an updated delta_t
DBMMK1::U EKFScheduler::get_control(double delta_t)
{
    auto control = this->_last_control_processed;
    control(2) = delta_t;
    return control;
}

void EKFScheduler::update_ekf(const DBMMK1::U control, const DBMMK1::Z measures)
{
}
