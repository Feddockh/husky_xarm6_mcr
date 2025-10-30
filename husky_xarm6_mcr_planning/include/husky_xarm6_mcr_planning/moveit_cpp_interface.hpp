/**
 * @file moveit_cpp_interface.hpp
 * @brief C++ interface to MoveIt for motion planning
 * 
 * This provides a C++ wrapper around MoveIt functionality that can be
 * extended or overridden. Python code can call into this via pybind11
 * bindings if needed.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <memory>
#include <string>
#include <vector>

namespace husky_xarm6_mcr_planning
{

/**
 * @brief C++ interface to MoveIt for planning and execution
 */
class MoveItCppInterface
{
public:
    /**
     * @brief Constructor
     * @param node ROS2 node
     * @param planning_group Name of the planning group (e.g., "xarm6_manipulator")
     */
    MoveItCppInterface(
        const rclcpp::Node::SharedPtr& node,
        const std::string& planning_group);

    /**
     * @brief Plan to a target pose
     * @param target_pose Target end-effector pose
     * @return true if planning succeeded
     */
    bool planToPose(const geometry_msgs::msg::Pose& target_pose);

    /**
     * @brief Execute the last planned trajectory
     * @return true if execution succeeded
     */
    bool execute();

    /**
     * @brief Plan and execute to target pose
     * @param target_pose Target end-effector pose
     * @return true if plan and execute succeeded
     */
    bool planAndExecute(const geometry_msgs::msg::Pose& target_pose);

    /**
     * @brief Get current end-effector pose
     * @return Current pose
     */
    geometry_msgs::msg::PoseStamped getCurrentPose();

    /**
     * @brief Check if a pose is reachable (has IK solution)
     * @param target_pose Pose to check
     * @return true if reachable
     */
    bool isReachable(const geometry_msgs::msg::Pose& target_pose);

    /**
     * @brief Set planning time
     * @param seconds Planning time in seconds
     */
    void setPlanningTime(double seconds);

    /**
     * @brief Set velocity scaling factor
     * @param factor Scaling factor (0.0 to 1.0)
     */
    void setVelocityScaling(double factor);

protected:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_;
    
    std::string planning_group_;
    moveit::planning_interface::MoveGroupInterface::Plan last_plan_;
};

} // namespace husky_xarm6_mcr_planning
