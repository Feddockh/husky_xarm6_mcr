#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

int main(int argc, char **argv)
{
    // Initialize ROS
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("plan_to_xyz");

    // Create MoveGroupInterface for the xArm planning group
    static const std::string PLANNING_GROUP = "xarm6";
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

    // Optional: allow replanning and set planning time
    move_group.allowReplanning(true);
    move_group.setPlanningTime(10.0);

    // Define the target pose in the planning frame
    geometry_msgs::msg::Pose target_pose;
    // Set your desired XYZ here:
    target_pose.position.x = -0.3;
    target_pose.position.y = 0.097;
    target_pose.position.z = 0.5; // Range is [0.5, 0.9]
    target_pose.orientation.x = -0.707;
    target_pose.orientation.y = 0.000;
    target_pose.orientation.z = -0.000;
    target_pose.orientation.w = 0.707;

    move_group.setPoseTarget(target_pose);

    // Plan to the pose
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
        RCLCPP_INFO(node->get_logger(), "[plan_to_xyz] Planning succeeded.");
        move_group.execute(plan);
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "[plan_to_xyz] Planning failed.");
    }

    // Shutdown ROS
    rclcpp::shutdown();
    return success ? 0 : 1;
}