#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "mirena_common/msg/entity_list.hpp"

#define GPS_SUB_TOPIC "sensors/gps"
#define WSS_SUB_TOPIC "sensors/wss"
#define IMU_SUB_TOPIC "sensors/imu"
#define CAR_PUB_TOPIC "consensus/car"

class MirenaConsensusNode : public rclcpp::Node {
public:
    MirenaLidarNode() : Node("mirena_lidar") {
        subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            LIDAR_SUB_TOPIC, 10, std::bind(&MirenaLidarNode::cloud_callback, this, std::placeholders::_1));

        publisher_ = create_publisher<mirena_common::msg::EntityList>(ENTITY_PUB_TOPIC, 10);

        RCLCPP_INFO(this->get_logger(), "Lidar Processing Node initialized.");
    }

private:
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Convertir de ROS PointCloud2 a PCL PointCloud
        PointCloudPtr input_cloud(new PointCloud);
        pcl::fromROSMsg(*msg, *input_cloud);

        // Procesar la nube con ConeDetector
        ConeDetector detector;
        EntityListPtr entities = detector.get_entities(input_cloud);
        if(entities->size() == 0){return;}

        // Convertir la nube de conos a ROS PointCloud2
        mirena_common::msg::EntityList output_msg;
        output_msg.header = msg->header;  // Mantener el frame y timestamp original
        output_msg.entities.resize(entities->size());
        for(size_t i = 0; i < entities->size(); i++){
            Entity &local_entity = entities->at(i);
            auto& ros_entity = output_msg.entities.at(i);

            ros_entity.position.set__x(local_entity._x);
            ros_entity.position.set__y(local_entity._y);
            ros_entity.set__type(local_entity._type);
            ros_entity.set__confidence(local_entity._confidence);
        }

        // Publicar los conos detectados
        publisher_->publish(output_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<mirena_common::msg::EntityList>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MirenaLidarNode>());
    rclcpp::shutdown();
    return 0;
}
