#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("get_eef_tf");

    // Parameters (so you can test other frames without recompiling)
    const std::string target_frame =
        node->declare_parameter<std::string>("target_frame", "world");
    const std::string source_frame =
        node->declare_parameter<std::string>("source_frame", "link_eef");
    const double timeout_s = node->declare_parameter<double>("timeout_s", 2.0);

    // TF buffer/listener
    tf2_ros::Buffer buffer(node->get_clock());
    tf2_ros::TransformListener listener(buffer);

    // Optionally match server time if you use sim time
    // node->set_parameter(rclcpp::Parameter("use_sim_time", true));

    // Give TF some time to populate
    rclcpp::sleep_for(std::chrono::milliseconds(200));

    geometry_msgs::msg::TransformStamped tf;
    try
    {
        tf = buffer.lookupTransform(target_frame, source_frame,
                            rclcpp::Time(0),                                  // latest
                            rclcpp::Duration::from_seconds(timeout_s));       // timeout
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR(node->get_logger(),
                     "TF lookup failed %s -> %s: %s",
                     target_frame.c_str(), source_frame.c_str(), ex.what());
        rclcpp::shutdown();
        return 1;
    }

    const auto &t = tf.transform.translation;
    const auto &q = tf.transform.rotation;

    RCLCPP_INFO(node->get_logger(), "Transform %s -> %s",
                target_frame.c_str(), source_frame.c_str());
    RCLCPP_INFO(node->get_logger(), "Translation: [%.3f, %.3f, %.3f]",
                t.x, t.y, t.z);
    RCLCPP_INFO(node->get_logger(), "Quaternion (xyzw): [%.6f, %.6f, %.6f, %.6f]",
                q.x, q.y, q.z, q.w);

    // Also print RPY
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    RCLCPP_INFO(node->get_logger(), "RPY (rad): [%.6f, %.6f, %.6f]", roll, pitch, yaw);

    rclcpp::shutdown();
    return 0;
}
