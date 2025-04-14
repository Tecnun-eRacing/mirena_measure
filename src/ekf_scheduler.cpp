#include "ekf_scheduler.hpp"

#include <rclcpp/rclcpp.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
#include <tf2/utils.h>
#include <cmath>

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
    GPSPos gps_pos(msg->altitude, msg->latitude);
    CartesianCoords coords;
    if(this->gps_origin.has_value()){
        coords = gps_pos.to_cartesian(this->gps_origin.value());
    } else {
        this->gps_origin = gps_pos;
        coords = CartesianCoords{0, 0};
    }
    double p_x = coords.x;
    double p_y = coords.y;
    // Once unpacked

    // We predict the measures and overwrite the ones we know
    DBMMK1::U control = this->get_control(delta_t);
    DBMMK1::Z inferred_measure = this->_ekf.predict_measures(control);
    inferred_measure(0) = p_x;
    inferred_measure(1) = p_y;

    this->_ekf.update(control, inferred_measure);
    this->_last_ekf_update = msg->header.stamp;
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

    // UNPACK THE MESSAGE
    tf2::Quaternion tf2_quat;
    tf2::fromMsg(msg->orientation, tf2_quat);
    double phi = tf2::getYaw(tf2_quat);
    double omega = msg->angular_velocity.z;

    // We predict the measures and overwrite the ones we know
    DBMMK1::U control = this->get_control(delta_t);
    DBMMK1::Z inferred_measure = this->_ekf.predict_measures(control);
    inferred_measure(2) = phi;
    inferred_measure(3) = omega;

    this->_ekf.update(control, inferred_measure);
    this->_last_ekf_update = msg->header.stamp;
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

void EKFScheduler::receive_control(const mirena_common::msg::CarControl::SharedPtr msg)
{
    this->_last_control_processed(0) = msg->acceleration;
    this->_last_control_processed(1) = msg->steer_angle;
}

mirena_common::msg::Car EKFScheduler::predict_state()
{
    rclcpp::Time now = this->_clock->now();
    double delta_t = now.seconds() - this->_last_wss_update.seconds();
    DBMMK1::X predicted_state_raw = this->_ekf.predict_state(this->get_control(delta_t));
    DBMMK1::StateAcessor predicted_state(predicted_state_raw);

    static_assert(false, "CAR MESSAGE CONVERSION NOT IMPLEMENTED IN EKF_SCHEDULER");
    // Populate the car message from the state:
    mirena_common::msg::Car msg;
    msg.header.set__frame_id(FIXED_FRAME);
    msg.header.set__stamp(now.to_msg());

    msg.pose.position.set__x(predicted_state.p_x());
    msg.pose.position.set__y(predicted_state.p_y());
    msg.pose.position.set__z(0);
    tf2::Quaternion q;
    q.setRPY(0, 0, predicted_state.phi());  // RPY in radians
    geometry_msgs::msg::Quaternion q_msg;
    tf2::convert(q, q_msg);
    msg.pose.set__orientation(q_msg);

    msg.velocity.linear.set__x(sin(predicted_state.phi())*predicted_state.u()+cos(predicted_state.phi())*predicted_state.v());
    msg.velocity.linear.set__y(cos(predicted_state.phi())*predicted_state.u()+sin(predicted_state.phi())*predicted_state.v());
    msg.velocity.linear.set__z(0);
    msg.velocity.angular.set__x(0);
    msg.velocity.angular.set__y(0);
    msg.velocity.angular.set__z(predicted_state.omega());
     
    return msg;
}

// Returns the last control value used but with an updated delta_t
DBMMK1::U EKFScheduler::get_control(double delta_t)
{
    auto control = this->_last_control_processed;
    control(2) = delta_t;
    return control;
}

CartesianCoords GPSPos::to_cartesian(GPSPos &origin)
{
    const double EARTH_RADIUS_M = 6378137.0; 
    const double DEG_TO_RAD = M_PI/180.0;
        // Convert to radians
    double lat_rad = this->_latitude * DEG_TO_RAD;
    double lon_rad = this->_longitude * DEG_TO_RAD;
    double lat_o_rad = origin._latitude * DEG_TO_RAD;
    double lon_o_rad = origin._longitude * DEG_TO_RAD;

    // Deltas in radians
    double dlat = lat_rad - lat_o_rad;
    double dlon = lon_rad - lon_o_rad;

    // Approximated distances (x: east, y: north)
    double x = EARTH_RADIUS_M * dlon * cos(lat_o_rad);
    double y = EARTH_RADIUS_M * dlat;

    return CartesianCoords{x, y};
}
