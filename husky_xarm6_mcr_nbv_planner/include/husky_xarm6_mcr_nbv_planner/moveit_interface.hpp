#ifndef HUSKY_XARM6_MCR_NBV_PLANNER_MOVEIT_INTERFACE_HPP
#define HUSKY_XARM6_MCR_NBV_PLANNER_MOVEIT_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Geometry>
#include <vector>

class MoveItInterface
{
public:
    MoveItInterface(const std::shared_ptr<rclcpp::Node> &node, const std::string &group_name);

    // Validation methods
    bool isPSMValid(bool verbose = true) const;
    bool isMoveGroupValid(bool verbose = true) const;

    // Joint model group retrieval
    bool getJointModelGroup(moveit::core::JointModelGroupConstPtr &jmg_out)
    {
        planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
        const moveit::core::RobotModelConstPtr &robot_model = scene->getRobotModel();
        return getJointModelGroup(jmg_out, robot_model);
    }

    bool getJointModelGroup(moveit::core::JointModelGroupConstPtr &jmg_out,
                            const moveit::core::RobotModelConstPtr &robot_model);

    // Joint state retrieval
    bool getCurrentJointAngles(std::vector<double> &joints_out,
                               const moveit::core::JointModelGroupConstPtr &jmg);

    // Joint position validation
    bool validateJointPositions(const std::vector<double> &joint_positions);

    // Collision checking
    bool isStateValid(const std::vector<double> &joint_positions)
    {
        planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
        moveit::core::RobotState state = scene->getCurrentState();
        return isStateValid(joint_positions, scene, state);
    }

    bool isStateValid(const std::vector<double> &joint_positions,
                      planning_scene_monitor::LockedPlanningSceneRO &scene,
                      moveit::core::RobotState &state);

    // Motion planning
    bool planToJointGoal(const std::vector<double> &joint_positions,
                         moveit::planning_interface::MoveGroupInterface::Plan &plan);

    // Trajectory execution
    bool execute(const moveit::planning_interface::MoveGroupInterface::Plan &plan);

    // Combined plan and execute
    bool planAndExecute(const std::vector<double> &joint_positions);

    // Inverse kinematics
    std::vector<double> computeIK(const std::vector<double> &seed_positions,
                                  const geometry_msgs::msg::Pose &target_pose,
                                  double timeout = 0.1, int attempts = 5);

    // IK configuration
    double getIKTimeout() const;
    void setIKTimeout(double timeout);

    // Planning configuration
    std::string getPlanningPipelineId() const;
    void setPlanningPipelineId(const std::string &pipeline_id);
    std::string getPlannerId() const;
    void setPlannerId(const std::string &planner_id);
    double getPlanningTime() const;
    void setPlanningTime(double seconds);
    int getNumPlanningAttempts() const;
    void setNumPlanningAttempts(int num_attempts);

    // Frame configuration
    std::string getPoseReferenceFrame() const;
    void setPoseReferenceFrame(const std::string &frame);
    std::string getEndEffectorLink() const;
    void setEndEffectorLink(const std::string &link);

    // Robot structure queries
    std::vector<std::string> getManipulatorLinks() const;
    std::vector<std::string> getManipulatorJointNames() const;
    std::vector<std::string> getAllRobotLinkNames() const;
    std::vector<std::string> getAllRobotJointNames() const;

    // Transform queries
    bool getTransformBetweenLinks(const std::string &from_link,
                                  const std::string &to_link,
                                  Eigen::Isometry3d &transform) const;
    bool getLinkPose(const std::string &link_name,
                     geometry_msgs::msg::Pose &pose) const;

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::string group_name_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm_;
    
    // Cached configuration values (MoveGroupInterface doesn't provide getters)
    double ik_timeout_{0.1};
    std::string planning_pipeline_id_;
    std::string planner_id_;
    double planning_time_{5.0};
    int num_planning_attempts_{10};
    std::string pose_reference_frame_;
    std::string end_effector_link_;
};

#endif
