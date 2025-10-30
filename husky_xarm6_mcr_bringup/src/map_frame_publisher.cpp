/**
 * @file map_frame_publisher.cpp
 * @brief Publishes static transform from map to odom frame
 * 
 * For simulation with perfect odometry, publishes identity transform.
 * In real robots, this would be replaced by a localization system (AMCL, etc.)
 */

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class MapFramePublisher : public rclcpp::Node
{
public:
    MapFramePublisher() : Node("map_frame_publisher")
    {
        // Declare parameters
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<std::string>("odom_frame", "odom");
        
        // Get parameters
        this->get_parameter("map_frame", map_frame_);
        this->get_parameter("odom_frame", odom_frame_);
        
        // Create static transform broadcaster
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        
        // Publish the static transform
        publish_static_transform();
        
        RCLCPP_INFO(this->get_logger(), 
            "Publishing static transform: %s -> %s", 
            map_frame_.c_str(), odom_frame_.c_str());
    }

private:
    void publish_static_transform()
    {
        geometry_msgs::msg::TransformStamped static_transform;
        
        static_transform.header.stamp = this->get_clock()->now();
        static_transform.header.frame_id = map_frame_;
        static_transform.child_frame_id = odom_frame_;
        
        // Identity transform (map and odom aligned at start)
        static_transform.transform.translation.x = 0.0;
        static_transform.transform.translation.y = 0.0;
        static_transform.transform.translation.z = 0.0;
        
        static_transform.transform.rotation.x = 0.0;
        static_transform.transform.rotation.y = 0.0;
        static_transform.transform.rotation.z = 0.0;
        static_transform.transform.rotation.w = 1.0;
        
        tf_static_broadcaster_->sendTransform(static_transform);
    }

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    std::string map_frame_;
    std::string odom_frame_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapFramePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
