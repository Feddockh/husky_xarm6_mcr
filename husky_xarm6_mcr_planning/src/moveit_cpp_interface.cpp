/**
 * @file moveit_cpp_interface.cpp
 * @brief Implementation of C++ MoveIt interface
 */

#include "husky_xarm6_mcr_planning/moveit_cpp_interface.hpp"

namespace husky_xarm6_mcr_planning
{

MoveItCppInterface::MoveItCppInterface(
    const rclcpp::Node::SharedPtr& node,
    const std::string& planning_group)
    : node_(node), planning_group_(planning_group)
{
    // Initialize MoveGroupInterface
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        node_, planning_group_);
    
    // Initialize PlanningSceneInterface
    planning_scene_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    
    RCLCPP_INFO(node_->get_logger(), 
                "MoveItCppInterface initialized for planning group: %s", 
                planning_group_.c_str());
    
    // Log current state
    RCLCPP_INFO(node_->get_logger(), "Reference frame: %s", 
                move_group_->getPlanningFrame().c_str());
    RCLCPP_INFO(node_->get_logger(), "End effector link: %s", 
                move_group_->getEndEffectorLink().c_str());
}

bool MoveItCppInterface::planToPose(const geometry_msgs::msg::Pose& target_pose)
{
    move_group_->setPoseTarget(target_pose);
    
    bool success = (move_group_->plan(last_plan_) == 
                    moveit::core::MoveItErrorCode::SUCCESS);
    
    if (success)
    {
        RCLCPP_INFO(node_->get_logger(), "Planning succeeded");
    }
    else
    {
        RCLCPP_WARN(node_->get_logger(), "Planning failed");
    }
    
    return success;
}

bool MoveItCppInterface::execute()
{
    bool success = (move_group_->execute(last_plan_) == 
                    moveit::core::MoveItErrorCode::SUCCESS);
    
    if (success)
    {
        RCLCPP_INFO(node_->get_logger(), "Execution succeeded");
    }
    else
    {
        RCLCPP_ERROR(node_->get_logger(), "Execution failed");
    }
    
    return success;
}

bool MoveItCppInterface::planAndExecute(const geometry_msgs::msg::Pose& target_pose)
{
    if (!planToPose(target_pose))
    {
        return false;
    }
    
    return execute();
}

geometry_msgs::msg::PoseStamped MoveItCppInterface::getCurrentPose()
{
    return move_group_->getCurrentPose();
}

bool MoveItCppInterface::isReachable(const geometry_msgs::msg::Pose& target_pose)
{
    // Quick IK check without full planning
    move_group_->setPoseTarget(target_pose);
    
    // Set very short planning time for quick check
    double original_time = move_group_->getPlanningTime();
    move_group_->setPlanningTime(0.5);
    
    moveit::planning_interface::MoveGroupInterface::Plan temp_plan;
    bool success = (move_group_->plan(temp_plan) == 
                    moveit::core::MoveItErrorCode::SUCCESS);
    
    // Restore original planning time
    move_group_->setPlanningTime(original_time);
    
    return success;
}

void MoveItCppInterface::setPlanningTime(double seconds)
{
    move_group_->setPlanningTime(seconds);
    RCLCPP_DEBUG(node_->get_logger(), "Planning time set to %.2f seconds", seconds);
}

void MoveItCppInterface::setVelocityScaling(double factor)
{
    if (factor < 0.0 || factor > 1.0)
    {
        RCLCPP_WARN(node_->get_logger(), 
                    "Velocity scaling factor %.2f out of range [0.0, 1.0], clamping", 
                    factor);
        factor = std::max(0.0, std::min(1.0, factor));
    }
    
    move_group_->setMaxVelocityScalingFactor(factor);
    RCLCPP_DEBUG(node_->get_logger(), "Velocity scaling set to %.2f", factor);
}

} // namespace husky_xarm6_mcr_planning
