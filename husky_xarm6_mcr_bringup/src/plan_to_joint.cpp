#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("plan_to_joint");
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);

    // Planning group (matches what your server prints in RViz: "xarm6")
    const std::string group_name = "xarm6";

    // Construct MoveGroupInterface
    moveit::planning_interface::MoveGroupInterface mgi(node, group_name);

    // Suggested basic planning settings
    mgi.setStartStateToCurrentState();
    mgi.setPlanningTime(5.0);
    mgi.setNumPlanningAttempts(5);

    // Get the group joint names so we don’t hardcode them
    const auto joint_names = mgi.getJointNames();
    RCLCPP_INFO(node->get_logger(), "Group '%s' has %zu joints.", group_name.c_str(), joint_names.size());
    for (const auto &j : joint_names)
        RCLCPP_INFO(node->get_logger(), "  %s", j.c_str());

    // ---- Target joint values ----
    // Example target close to a neutral pose; adjust as needed for your setup.
    // The vector size must match joint_names.size(). For xarm6 this should be 6.
    std::vector<double> target;
    target.reserve(joint_names.size());

    // Example values (degrees): [0.0, -90.0, -90.0, 90.0, 90.0, 90.0]
    // These are often a safe starting point; tweak if your environment needs different angles.
    target = {0.0, -1.5708, -1.5708, 1.5708, 1.5708, 1.5708}; // radians

    if (target.size() != joint_names.size())
    {
        RCLCPP_ERROR(node->get_logger(), "Target size (%zu) != number of joints in group (%zu).",
                     target.size(), joint_names.size());
        rclcpp::shutdown();
        return 1;
    }

    // Set the joint-value target using the discovered joint order
    mgi.setJointValueTarget(target);

    // Plan (no execution)
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto rc = mgi.plan(plan);

    if (rc == moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "[plan_to_joint_state] Planning succeeded.");
        mgi.execute(plan);
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "[plan_to_joint_state] Planning failed (code %d).", rc.val);
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
