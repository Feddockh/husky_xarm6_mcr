#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("sanity_check");

    // Planning group can be passed via ROS param; default matches xarm6 with prefix "xarm_"
    std::string planning_group = node->declare_parameter<std::string>("planning_group", "xarm6");
    double server_wait = node->declare_parameter<double>("server_wait", 10.0);

    // Construct MGI (this will also wait for the /move_group action server)
    moveit::planning_interface::MoveGroupInterface mgi(node, planning_group);

    // Print a few basics
    RCLCPP_INFO(node->get_logger(), "Planning group: %s", planning_group.c_str());
    RCLCPP_INFO(node->get_logger(), "Planning frame: %s", mgi.getPlanningFrame().c_str());
    RCLCPP_INFO(node->get_logger(), "End effector link: %s", mgi.getEndEffectorLink().c_str());

    const auto joint_names = mgi.getJointNames();
    std::string names;
    for (const auto &n : joint_names)
        names += n + " ";
    RCLCPP_INFO(node->get_logger(), "Group joint names: %s", names.c_str());

    rclcpp::shutdown();
    return 0;
}
