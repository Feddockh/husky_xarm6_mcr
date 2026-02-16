#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Geometry>
#include <vector>

namespace husky_xarm6_mcr_nbv_planner
{

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

        // Simplified joint angle retrieval without requiring JMG
        bool getCurrentJointAngles(std::vector<double> &joints_out);

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

        // Forward kinematics
        // Returns pose in reference_frame (default: robot base_link)
        bool getEndEffectorPose(const std::vector<double> &joint_positions,
                                Eigen::Isometry3d &ee_pose_out,
                                const std::string &reference_frame = "") const;

        bool getEndEffectorPose(const std::vector<double> &joint_positions,
                                geometry_msgs::msg::Pose &ee_pose_out,
                                const std::string &reference_frame = "") const;

        // Inverse kinematics
        // Target pose is interpreted in reference_frame (default: robot base_link)
        std::vector<double> computeIK(const std::vector<double> &seed_positions,
                                      const geometry_msgs::msg::Pose &target_pose,
                                      double timeout = 0.1, int attempts = 5,
                                      const std::string &reference_frame = "");
        
        std::vector<double> computeIK(const std::vector<double> &seed_positions,
                                      const Eigen::Vector3d &target_position,
                                      const std::array<double, 4> &target_orientation,
                                      double timeout = 0.1, int attempts = 5,
                                      const std::string &reference_frame = "");

        // Validate IK solution by checking FK error
        bool validateIKSolution(const std::vector<double> &joint_angles,
                                const Eigen::Isometry3d &target_pose,
                                double &pos_error_out,
                                double &angular_error_out,
                                double pos_threshold = 0.01,
                                double angular_threshold = 0.087) const;

        // Transformation from end-effector to camera
        bool getEEToCameraTransform(const std::string &camera_link,
                                    Eigen::Isometry3d &T_ee_cam_out) const;

        // Camera pose to end-effector pose
        // All poses interpreted/returned in reference_frame (default: robot base_link)
        bool cameraPoseToEEPose(const geometry_msgs::msg::Pose &cam_pose,
                                const std::string &camera_link,
                                geometry_msgs::msg::Pose &ee_pose_out,
                                const std::string &reference_frame = "") const;
        
        bool cameraPoseToEEPose(const Eigen::Vector3d &cam_position,
                                const std::array<double, 4> &cam_orientation,
                                const std::string &camera_link,
                                Eigen::Vector3d &ee_position_out,
                                std::array<double, 4> &ee_orientation_out,
                                const std::string &reference_frame = "") const;

        // IK configuration
        double getIKTimeout() const;
        void setIKTimeout(double timeout);

        // Planning configuration
        std::string getPlanningPipelineId() const;
        void setPlanningPipelineId(const std::string pipeline_id);
        std::string getPlannerId() const;
        void setPlannerId(const std::string planner_id);
        double getPlanningTime() const;
        void setPlanningTime(double seconds);
        int getNumPlanningAttempts() const;
        void setNumPlanningAttempts(int num_attempts);

        // Velocity and acceleration scaling
        double getMaxVelocityScalingFactor() const;
        void setMaxVelocityScalingFactor(double scale);
        double getMaxAccelerationScalingFactor() const;
        void setMaxAccelerationScalingFactor(double scale);

        // Path constraints
        void setOrientationConstraints(const std::string& link_name, 
                                      const geometry_msgs::msg::Quaternion& target_orientation,
                                      double tolerance_roll, double tolerance_pitch, double tolerance_yaw);
        void clearPathConstraints();

        // Frame configuration
        std::string getPoseReferenceFrame() const;
        void setPoseReferenceFrame(const std::string &frame);
        std::string getEndEffectorLink() const;
        void setEndEffectorLink(const std::string &link);
        
        // Default reference frame for cartesian operations
        std::string getDefaultReferenceFrame() const;
        void setDefaultReferenceFrame(const std::string &frame);

        // Robot structure queries
        std::vector<std::string> getManipulatorLinks() const;
        std::vector<std::string> getManipulatorJointNames() const;
        std::vector<std::string> getAllRobotLinkNames() const;
        std::vector<std::string> getAllRobotJointNames() const;

        // Transform queries
        bool getTransformBetweenLinks(const std::string &from_link,
                                      const std::string &to_link,
                                      Eigen::Isometry3d &transform) const;
        
        // Returns pose of link_name in reference_frame (default: robot base_link)
        bool getLinkPose(const std::string &link_name,
                         geometry_msgs::msg::Pose &pose,
                         const std::string &reference_frame = "") const;
        
        bool getLinkPose(const std::string &link_name,
                         Eigen::Vector3d &position,
                         std::array<double, 4> &orientation,
                         const std::string &reference_frame = "") const;

        // Allow access to PSM for workspace computation
        std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> getPlanningSceneMonitor() const { return psm_; }

    private:
        std::shared_ptr<rclcpp::Node> node_;
        std::string group_name_;
        std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
        std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm_;

        // Cached configuration values (MoveGroupInterface doesn't provide getters)
        double ik_timeout_{0.1};
        std::string planning_pipeline_id_;
        std::string planner_id_{"RRTConnect"};
        double planning_time_{3.0};
        int num_planning_attempts_{5};
        double max_velocity_scaling_factor_{0.1};
        double max_acceleration_scaling_factor_{0.1};
        std::string pose_reference_frame_;
        std::string end_effector_link_;
        std::string default_reference_frame_;  // Default for cartesian operations (initialized from robot model)
    };

} // namespace husky_xarm6_mcr_nbv_planner