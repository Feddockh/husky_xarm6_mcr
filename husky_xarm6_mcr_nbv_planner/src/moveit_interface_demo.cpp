#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "husky_xarm6_mcr_nbv_planner/moveit_interface.hpp"

using namespace husky_xarm6_mcr_nbv_planner;

void waitForSimClock(const std::shared_ptr<rclcpp::Node> &node)
{
    RCLCPP_INFO(node->get_logger(), "Waiting for /clock...");
    auto clock_sub = node->create_subscription<rosgraph_msgs::msg::Clock>(
        "/clock", 10, [](const rosgraph_msgs::msg::Clock::SharedPtr) {});

    rclcpp::Rate rate(10);
    auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        if (node->now().seconds() > 0.0)
        {
            RCLCPP_INFO(node->get_logger(), "Clock synchronized at %.2f seconds",
                        node->now().seconds());
            break;
        }
        auto elapsed = std::chrono::steady_clock::now() - start;
        if (elapsed > std::chrono::seconds(10))
        {
            RCLCPP_WARN(node->get_logger(), "Timeout waiting for /clock. Proceeding anyway...");
            break;
        }
        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>(
        "moveit_interface_node",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Wait for sim time if needed
    if (node->get_parameter("use_sim_time").as_bool())
    {
        waitForSimClock(node);
    }

    // Initialize MoveIt interface
    auto interface = std::make_shared<MoveItInterface>(node, "xarm6_manipulator");

    // Wait for move_group
    RCLCPP_INFO(node->get_logger(), "Waiting for move_group action server...");
    rclcpp::sleep_for(std::chrono::seconds(2));
    rclcpp::spin_some(node);

    // Get joint model group
    moveit::core::JointModelGroupConstPtr jmg;
    if (!interface->getJointModelGroup(jmg))
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to get joint model group!");
        return 1;
    }

    // Get current joint state
    RCLCPP_INFO(node->get_logger(), "\n=== Current Joint State ===");
    std::vector<double> current_joints;
    if (!interface->getCurrentJointAngles(current_joints, jmg))
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to get current joint state!");
        return 1;
    }

    std::string joint_str;
    for (double j : current_joints)
        joint_str += std::to_string(j) + " ";
    RCLCPP_INFO(node->get_logger(), "Current: [%s]", joint_str.c_str());

    // Display configuration info
    RCLCPP_INFO(node->get_logger(), "\n=== Configuration ===");
    std::string pipeline = interface->getPlanningPipelineId();
    std::string planner = interface->getPlannerId();
    RCLCPP_INFO(node->get_logger(), "Planning Pipeline: %s", 
                pipeline.empty() ? "(default)" : pipeline.c_str());
    RCLCPP_INFO(node->get_logger(), "Planner ID: %s", 
                planner.empty() ? "(none set)" : planner.c_str());
    RCLCPP_INFO(node->get_logger(), "Planning Time: %.2fs", interface->getPlanningTime());
    RCLCPP_INFO(node->get_logger(), "Planning Attempts: %d", interface->getNumPlanningAttempts());
    RCLCPP_INFO(node->get_logger(), "Pose Reference Frame: %s", interface->getPoseReferenceFrame().c_str());
    RCLCPP_INFO(node->get_logger(), "End Effector Link: %s", interface->getEndEffectorLink().c_str());

    // Display robot structure
    RCLCPP_INFO(node->get_logger(), "\n=== Robot Structure ===");
    auto manip_links = interface->getManipulatorLinks();
    auto manip_joints = interface->getManipulatorJointNames();
    RCLCPP_INFO(node->get_logger(), "Manipulator Group: 'xarm6_manipulator'");
    RCLCPP_INFO(node->get_logger(), "  Links (%zu):", manip_links.size());
    for (size_t i = 0; i < std::min(manip_links.size(), size_t(5)); ++i)
        RCLCPP_INFO(node->get_logger(), "    - %s", manip_links[i].c_str());
    if (manip_links.size() > 5)
        RCLCPP_INFO(node->get_logger(), "    ... and %zu more", manip_links.size() - 5);
    
    RCLCPP_INFO(node->get_logger(), "  Joints (%zu):", manip_joints.size());
    for (size_t i = 0; i < std::min(manip_joints.size(), size_t(5)); ++i)
        RCLCPP_INFO(node->get_logger(), "    - %s", manip_joints[i].c_str());
    if (manip_joints.size() > 5)
        RCLCPP_INFO(node->get_logger(), "    ... and %zu more", manip_joints.size() - 5);
    
    // Get end effector pose
    geometry_msgs::msg::Pose ee_pose;
    if (interface->getLinkPose(interface->getEndEffectorLink(), ee_pose))
    {
        RCLCPP_INFO(node->get_logger(), "  End Effector Pose: [%.3f, %.3f, %.3f]",
                    ee_pose.position.x, ee_pose.position.y, ee_pose.position.z);
    }

    // Define target pose
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.3;
    target_pose.position.y = -0.3;
    target_pose.position.z = 0.8;
    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = -0.707;
    target_pose.orientation.z = 0.707;
    target_pose.orientation.w = 0.0;

    RCLCPP_INFO(node->get_logger(), "\n=== Computing IK ===");
    RCLCPP_INFO(node->get_logger(), "Target: x=%.2f, y=%.2f, z=%.2f",
                target_pose.position.x, target_pose.position.y, target_pose.position.z);

    auto ik_solution = interface->computeIK(current_joints, target_pose, 0.5);

    if (ik_solution.empty())
    {
        RCLCPP_WARN(node->get_logger(), "No valid IK solution found.");
    }
    else
    {
        std::string ik_str;
        for (double j : ik_solution)
            ik_str += std::to_string(j) + " ";
        RCLCPP_INFO(node->get_logger(), "IK Solution: [%s]", ik_str.c_str());

        // Validate solution
        if (interface->isStateValid(ik_solution))
        {
            RCLCPP_INFO(node->get_logger(), "IK solution is valid (collision-free).");

            // Plan to IK solution
            RCLCPP_INFO(node->get_logger(), "\n=== Planning Motion ===");
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            if (interface->planToJointGoal(ik_solution, plan))
            {
                // Execute trajectory
                RCLCPP_INFO(node->get_logger(), "\n=== Executing Trajectory ===");
                if (interface->execute(plan))
                {
                    RCLCPP_INFO(node->get_logger(), "\n=== SUCCESS: Motion completed! ===");
                }
                else
                {
                    RCLCPP_ERROR(node->get_logger(), "Execution failed.");
                }
            }
            else
            {
                RCLCPP_WARN(node->get_logger(), "Planning failed.");
            }
        }
        else
        {
            RCLCPP_WARN(node->get_logger(), "IK solution is in collision.");
        }
    }

    RCLCPP_INFO(node->get_logger(), "\nNode will continue running. Press Ctrl+C to exit.");
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
