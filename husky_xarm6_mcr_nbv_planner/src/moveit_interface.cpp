#include "husky_xarm6_mcr_nbv_planner/moveit_interface.hpp"

#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <tf2_eigen/tf2_eigen.hpp>

namespace husky_xarm6_mcr_nbv_planner
{

    MoveItInterface::MoveItInterface(const std::shared_ptr<rclcpp::Node> &node,
                                     const std::string &group_name)
        : node_(node), group_name_(group_name)
    {
        move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_, group_name_);
        psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_, "robot_description");

        if (!isPSMValid())
            return;

        psm_->startSceneMonitor();
        psm_->startStateMonitor();
        psm_->startWorldGeometryMonitor();

        RCLCPP_INFO(node_->get_logger(), "Waiting for current robot state...");
        if (!psm_->waitForCurrentRobotState(node_->now(), 5.0))
        {
            RCLCPP_WARN(node_->get_logger(), "Timed out waiting for robot state.");
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "Robot state received successfully.");
        }

        planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
        auto world_objects = scene->getWorld()->getObjectIds();
        RCLCPP_INFO(node_->get_logger(), "Planning scene has %zu world objects", world_objects.size());
        for (const auto &obj_id : world_objects)
        {
            RCLCPP_INFO(node_->get_logger(), "  - %s", obj_id.c_str());
        }

        // Initialize cached configuration with MoveGroup defaults
        if (isMoveGroupValid(false))
        {
            pose_reference_frame_ = move_group_->getPoseReferenceFrame();
            end_effector_link_ = move_group_->getEndEffectorLink();
            planning_pipeline_id_ = move_group_->getPlanningPipelineId();
            planner_id_ = move_group_->getPlannerId();
            planning_time_ = move_group_->getPlanningTime();

            RCLCPP_DEBUG(node_->get_logger(), "Initialized config - Pipeline: '%s', Planner: '%s'",
                         planning_pipeline_id_.c_str(), planner_id_.c_str());

            // Set default planner if not already configured
            if (planner_id_.empty())
            {
                planner_id_ = "RRTConnect";
                move_group_->setPlannerId(planner_id_);
                RCLCPP_INFO(node_->get_logger(), "Set default planner to: %s", planner_id_.c_str());
            }

            // Set our default values
            move_group_->setNumPlanningAttempts(num_planning_attempts_);
        }
    }

    bool MoveItInterface::isPSMValid(bool verbose) const
    {
        bool valid = (psm_ && psm_->getPlanningScene());
        if (!valid && verbose)
            RCLCPP_ERROR(node_->get_logger(), "PlanningSceneMonitor is NOT valid.");
        return valid;
    }

    bool MoveItInterface::isMoveGroupValid(bool verbose) const
    {
        bool valid = (move_group_ != nullptr);
        if (!valid && verbose)
            RCLCPP_ERROR(node_->get_logger(), "MoveGroupInterface is NOT initialized.");
        return valid;
    }

    bool MoveItInterface::getJointModelGroup(moveit::core::JointModelGroupConstPtr &jmg_out,
                                             const moveit::core::RobotModelConstPtr &robot_model)
    {
        if (!robot_model)
        {
            RCLCPP_ERROR(node_->get_logger(), "RobotModel is null");
            return false;
        }

        const moveit::core::JointModelGroup *jmg_raw = robot_model->getJointModelGroup(group_name_);
        if (!jmg_raw)
        {
            RCLCPP_ERROR(node_->get_logger(), "JointModelGroup '%s' not found.", group_name_.c_str());
            return false;
        }

        // Store raw pointer in shared_ptr with no-op deleter since robot_model owns it
        jmg_out = moveit::core::JointModelGroupConstPtr(jmg_raw, [](const moveit::core::JointModelGroup *) {});
        return true;
    }

    bool MoveItInterface::getCurrentJointAngles(std::vector<double> &joints_out,
                                                const moveit::core::JointModelGroupConstPtr &jmg)
    {
        if (!isPSMValid())
            return false;

        planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
        const moveit::core::RobotState &current_state = scene->getCurrentState();
        current_state.copyJointGroupPositions(jmg.get(), joints_out);
        return true;
    }

    bool MoveItInterface::getCurrentJointAngles(std::vector<double> &joints_out)
    {
        moveit::core::JointModelGroupConstPtr jmg;
        if (!getJointModelGroup(jmg))
            return false;
        return getCurrentJointAngles(joints_out, jmg);
    }

    bool MoveItInterface::validateJointPositions(const std::vector<double> &joint_positions)
    {
        moveit::core::JointModelGroupConstPtr jmg;
        if (!getJointModelGroup(jmg))
            return false;

        const std::vector<std::string> &active_joint_names = jmg->getActiveJointModelNames();
        if (joint_positions.size() != active_joint_names.size())
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "joint_positions size %zu != expected %zu for group '%s'.",
                         joint_positions.size(), active_joint_names.size(), group_name_.c_str());
            return false;
        }
        return true;
    }

    bool MoveItInterface::isStateValid(const std::vector<double> &joint_positions,
                                       planning_scene_monitor::LockedPlanningSceneRO &scene,
                                       moveit::core::RobotState &state)
    {
        if (!isPSMValid())
            return false;

        moveit::core::JointModelGroupConstPtr jmg;
        if (!getJointModelGroup(jmg, state.getRobotModel()))
            return false;

        if (!validateJointPositions(joint_positions))
            return false;

        state.setJointGroupPositions(jmg.get(), joint_positions);
        state.update();

        collision_detection::CollisionRequest req;
        collision_detection::CollisionResult res;
        req.group_name = group_name_;
        scene->checkCollision(req, res, state);

        if (res.collision)
            RCLCPP_DEBUG(node_->get_logger(), "State is in collision");

        return !res.collision;
    }

    bool MoveItInterface::planToJointGoal(const std::vector<double> &joint_positions,
                                          moveit::planning_interface::MoveGroupInterface::Plan &plan)
    {
        if (!isMoveGroupValid() || !validateJointPositions(joint_positions))
            return false;

        move_group_->setJointValueTarget(joint_positions);
        moveit::core::MoveItErrorCode result = move_group_->plan(plan);

        if (result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(node_->get_logger(), "Planning succeeded. Trajectory: %zu waypoints, %.2fs",
                        plan.trajectory_.joint_trajectory.points.size(),
                        rclcpp::Duration(plan.trajectory_.joint_trajectory.points.back().time_from_start).seconds());
            return true;
        }

        RCLCPP_WARN(node_->get_logger(), "Planning failed (code: %d)", result.val);
        return false;
    }

    bool MoveItInterface::execute(const moveit::planning_interface::MoveGroupInterface::Plan &plan)
    {
        if (!isMoveGroupValid())
            return false;

        RCLCPP_INFO(node_->get_logger(), "Executing trajectory...");
        moveit::core::MoveItErrorCode result = move_group_->execute(plan);

        if (result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(node_->get_logger(), "Trajectory execution succeeded.");
            return true;
        }

        RCLCPP_ERROR(node_->get_logger(), "Trajectory execution failed (code: %d)", result.val);
        return false;
    }

    bool MoveItInterface::planAndExecute(const std::vector<double> &joint_positions)
    {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        return planToJointGoal(joint_positions, plan) && execute(plan);
    }

    std::vector<double> MoveItInterface::computeIK(const std::vector<double> &seed_positions,
                                                   const geometry_msgs::msg::Pose &target_pose,
                                                   double timeout, int attempts)
    {
        std::vector<double> ik_solution;

        if (!isPSMValid() || !validateJointPositions(seed_positions))
            return ik_solution;

        planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
        moveit::core::RobotState state = scene->getCurrentState();

        moveit::core::JointModelGroupConstPtr jmg;
        if (!getJointModelGroup(jmg, state.getRobotModel()))
            return ik_solution;

        state.setJointGroupPositions(jmg.get(), seed_positions);
        state.update();

        Eigen::Isometry3d target_eigen;
        tf2::fromMsg(target_pose, target_eigen);

        const std::string &ee_link = jmg->getLinkModelNames().back();

        bool ik_success = false;
        while (attempts > 0 && !ik_success)
        {
            ik_success = state.setFromIK(jmg.get(), target_eigen, ee_link, timeout);
            attempts--;
        }

        if (ik_success)
        {
            collision_detection::CollisionRequest req;
            collision_detection::CollisionResult res;
            req.group_name = group_name_;
            scene->checkCollision(req, res, state);

            if (!res.collision)
            {
                state.copyJointGroupPositions(jmg.get(), ik_solution);
                RCLCPP_DEBUG(node_->get_logger(), "IK solution found (collision-free).");
            }
            else
            {
                RCLCPP_WARN(node_->get_logger(), "IK solution found but in collision.");
            }
        }
        else
        {
            RCLCPP_DEBUG(node_->get_logger(), "IK solution not found.");
        }

        return ik_solution;
    }

    std::vector<double> MoveItInterface::computeIK(const std::vector<double> &seed_positions,
                                                   const Eigen::Vector3d &target_position,
                                                   const std::array<double, 4> &target_orientation,
                                                   double timeout, int attempts)
    {
        // Convert Eigen types to geometry_msgs::Pose
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = target_position.x();
        target_pose.position.y = target_position.y();
        target_pose.position.z = target_position.z();
        target_pose.orientation.x = target_orientation[0];
        target_pose.orientation.y = target_orientation[1];
        target_pose.orientation.z = target_orientation[2];
        target_pose.orientation.w = target_orientation[3];
        
        return computeIK(seed_positions, target_pose, timeout, attempts);
    }

    bool MoveItInterface::getEndEffectorPose(const std::vector<double> &joint_positions,
                                             Eigen::Isometry3d &ee_pose_out) const
    {
        if (!isMoveGroupValid(false))
        {
            RCLCPP_WARN(node_->get_logger(), "MoveGroup not initialized");
            return false;
        }

        auto current_state = move_group_->getCurrentState();
        auto jmg = current_state->getJointModelGroup(group_name_);

        if (!jmg)
        {
            RCLCPP_ERROR(node_->get_logger(), "Joint model group not found");
            return false;
        }

        current_state->setJointGroupPositions(jmg, joint_positions);
        const Eigen::Isometry3d &pose = current_state->getGlobalLinkTransform(move_group_->getEndEffectorLink());
        ee_pose_out = pose;

        return true;
    }

    bool MoveItInterface::getEndEffectorPose(const std::vector<double> &joint_positions,
                                             geometry_msgs::msg::Pose &ee_pose_out) const
    {
        Eigen::Isometry3d pose_eigen;
        if (!getEndEffectorPose(joint_positions, pose_eigen))
        {
            return false;
        }

        ee_pose_out.position.x = pose_eigen.translation().x();
        ee_pose_out.position.y = pose_eigen.translation().y();
        ee_pose_out.position.z = pose_eigen.translation().z();

        Eigen::Quaterniond quat(pose_eigen.rotation());
        ee_pose_out.orientation.x = quat.x();
        ee_pose_out.orientation.y = quat.y();
        ee_pose_out.orientation.z = quat.z();
        ee_pose_out.orientation.w = quat.w();

        return true;
    }

    bool MoveItInterface::validateIKSolution(const std::vector<double> &joint_angles,
                                             const Eigen::Isometry3d &target_pose,
                                             double &pos_error_out,
                                             double &angular_error_out,
                                             double pos_threshold,
                                             double angular_threshold) const
    {
        Eigen::Isometry3d computed_pose;
        if (!getEndEffectorPose(joint_angles, computed_pose))
        {
            return false;
        }

        // Position error
        Eigen::Vector3d pos_error = computed_pose.translation() - target_pose.translation();
        pos_error_out = pos_error.norm();

        // Angular error (rotation difference)
        Eigen::Quaterniond q1(computed_pose.rotation());
        Eigen::Quaterniond q2(target_pose.rotation());
        angular_error_out = q1.angularDistance(q2);

        return (pos_error_out < pos_threshold && angular_error_out < angular_threshold);
    }

    bool MoveItInterface::getEEToCameraTransform(const std::string &camera_link,
                                                 Eigen::Isometry3d &T_ee_cam_out) const
    {
        if (!isPSMValid(false))
            return false;

        const std::string ee_link = getEndEffectorLink();
        if (ee_link.empty())
        {
            RCLCPP_ERROR(node_->get_logger(), "End effector link is empty. Did MoveGroupInterface initialize correctly?");
            return false;
        }

        planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
        const moveit::core::RobotState &state = scene->getCurrentState();

        if (!state.knowsFrameTransform(ee_link))
        {
            RCLCPP_ERROR(node_->get_logger(), "Unknown EE frame: %s", ee_link.c_str());
            return false;
        }
        if (!state.knowsFrameTransform(camera_link))
        {
            RCLCPP_ERROR(node_->get_logger(), "Unknown camera frame: %s", camera_link.c_str());
            return false;
        }

        // Global transforms
        const Eigen::Isometry3d T_world_ee = state.getGlobalLinkTransform(ee_link);
        const Eigen::Isometry3d T_world_cam = state.getGlobalLinkTransform(camera_link);

        // Relative EE->camera
        T_ee_cam_out = T_world_ee.inverse() * T_world_cam;
        return true;
    }

    bool MoveItInterface::cameraPoseToEEPose(const geometry_msgs::msg::Pose &cam_pose_in_world,
                                             const std::string &camera_link,
                                             geometry_msgs::msg::Pose &ee_pose_out_world) const
    {
        Eigen::Isometry3d T_ee_cam;
        if (!getEEToCameraTransform(camera_link, T_ee_cam))
        {
            return false;
        }

        Eigen::Isometry3d T_world_cam;
        tf2::fromMsg(cam_pose_in_world, T_world_cam);

        const Eigen::Isometry3d T_world_ee = T_world_cam * T_ee_cam.inverse();
        ee_pose_out_world = tf2::toMsg(T_world_ee);
        return true;
    }

    bool MoveItInterface::cameraPoseToEEPose(const Eigen::Vector3d &cam_position,
                                             const std::array<double, 4> &cam_orientation,
                                             const std::string &camera_link,
                                             Eigen::Vector3d &ee_position_out,
                                             std::array<double, 4> &ee_orientation_out) const
    {
        // Convert Eigen types to geometry_msgs::Pose
        geometry_msgs::msg::Pose cam_pose;
        cam_pose.position.x = cam_position.x();
        cam_pose.position.y = cam_position.y();
        cam_pose.position.z = cam_position.z();
        cam_pose.orientation.x = cam_orientation[0];
        cam_pose.orientation.y = cam_orientation[1];
        cam_pose.orientation.z = cam_orientation[2];
        cam_pose.orientation.w = cam_orientation[3];
        
        geometry_msgs::msg::Pose ee_pose;
        if (!cameraPoseToEEPose(cam_pose, camera_link, ee_pose))
        {
            return false;
        }
        
        // Convert back to Eigen types
        ee_position_out.x() = ee_pose.position.x;
        ee_position_out.y() = ee_pose.position.y;
        ee_position_out.z() = ee_pose.position.z;
        ee_orientation_out[0] = ee_pose.orientation.x;
        ee_orientation_out[1] = ee_pose.orientation.y;
        ee_orientation_out[2] = ee_pose.orientation.z;
        ee_orientation_out[3] = ee_pose.orientation.w;
        
        return true;
    }

    // IK configuration
    double MoveItInterface::getIKTimeout() const
    {
        return ik_timeout_;
    }

    void MoveItInterface::setIKTimeout(double timeout)
    {
        ik_timeout_ = timeout;
    }

    // Planning configuration
    std::string MoveItInterface::getPlanningPipelineId() const
    {
        if (!isMoveGroupValid(false))
            return planning_pipeline_id_;
        return move_group_->getPlanningPipelineId();
    }

    void MoveItInterface::setPlanningPipelineId(const std::string &pipeline_id)
    {
        planning_pipeline_id_ = pipeline_id;
        if (isMoveGroupValid(false))
            move_group_->setPlanningPipelineId(pipeline_id);
    }

    std::string MoveItInterface::getPlannerId() const
    {
        if (!isMoveGroupValid(false))
            return planner_id_;
        return move_group_->getPlannerId();
    }

    void MoveItInterface::setPlannerId(const std::string &planner_id)
    {
        planner_id_ = planner_id;
        if (isMoveGroupValid(false))
            move_group_->setPlannerId(planner_id);
    }

    double MoveItInterface::getPlanningTime() const
    {
        if (!isMoveGroupValid(false))
            return planning_time_;
        return move_group_->getPlanningTime();
    }

    void MoveItInterface::setPlanningTime(double seconds)
    {
        planning_time_ = seconds;
        if (isMoveGroupValid(false))
            move_group_->setPlanningTime(seconds);
    }

    int MoveItInterface::getNumPlanningAttempts() const
    {
        return num_planning_attempts_;
    }

    void MoveItInterface::setNumPlanningAttempts(int num_attempts)
    {
        num_planning_attempts_ = num_attempts;
        if (isMoveGroupValid(false))
            move_group_->setNumPlanningAttempts(num_attempts);
    }

    // Frame configuration
    std::string MoveItInterface::getPoseReferenceFrame() const
    {
        if (!isMoveGroupValid(false))
            return pose_reference_frame_;
        return move_group_->getPoseReferenceFrame();
    }

    void MoveItInterface::setPoseReferenceFrame(const std::string &frame)
    {
        pose_reference_frame_ = frame;
        if (isMoveGroupValid(false))
            move_group_->setPoseReferenceFrame(frame);
    }

    std::string MoveItInterface::getEndEffectorLink() const
    {
        if (!isMoveGroupValid(false))
            return end_effector_link_;
        return move_group_->getEndEffectorLink();
    }

    void MoveItInterface::setEndEffectorLink(const std::string &link)
    {
        end_effector_link_ = link;
        if (isMoveGroupValid(false))
            move_group_->setEndEffectorLink(link);
    }

    // Robot structure queries
    std::vector<std::string> MoveItInterface::getManipulatorLinks() const
    {
        std::vector<std::string> links;

        moveit::core::JointModelGroupConstPtr jmg;
        if (!const_cast<MoveItInterface *>(this)->getJointModelGroup(jmg))
            return links;

        return jmg->getLinkModelNames();
    }

    std::vector<std::string> MoveItInterface::getManipulatorJointNames() const
    {
        std::vector<std::string> joints;

        moveit::core::JointModelGroupConstPtr jmg;
        if (!const_cast<MoveItInterface *>(this)->getJointModelGroup(jmg))
            return joints;

        return jmg->getActiveJointModelNames();
    }

    std::vector<std::string> MoveItInterface::getAllRobotLinkNames() const
    {
        std::vector<std::string> links;

        if (!isPSMValid(false))
            return links;

        planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
        const moveit::core::RobotModelConstPtr &robot_model = scene->getRobotModel();

        const std::vector<const moveit::core::LinkModel *> &link_models = robot_model->getLinkModels();
        for (const auto *link : link_models)
        {
            links.push_back(link->getName());
        }

        return links;
    }

    std::vector<std::string> MoveItInterface::getAllRobotJointNames() const
    {
        std::vector<std::string> joints;

        if (!isPSMValid(false))
            return joints;

        planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
        const moveit::core::RobotModelConstPtr &robot_model = scene->getRobotModel();

        const std::vector<const moveit::core::JointModel *> &joint_models = robot_model->getJointModels();
        for (const auto *joint : joint_models)
        {
            joints.push_back(joint->getName());
        }

        return joints;
    }

    // Transform queries
    bool MoveItInterface::getTransformBetweenLinks(const std::string &from_link,
                                                   const std::string &to_link,
                                                   Eigen::Isometry3d &transform) const
    {
        if (!isPSMValid(false))
            return false;

        planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
        const moveit::core::RobotState &state = scene->getCurrentState();

        if (!state.knowsFrameTransform(from_link))
        {
            RCLCPP_ERROR(node_->get_logger(), "Unknown frame: %s", from_link.c_str());
            return false;
        }

        if (!state.knowsFrameTransform(to_link))
        {
            RCLCPP_ERROR(node_->get_logger(), "Unknown frame: %s", to_link.c_str());
            return false;
        }

        const Eigen::Isometry3d &from_transform = state.getGlobalLinkTransform(from_link);
        const Eigen::Isometry3d &to_transform = state.getGlobalLinkTransform(to_link);

        transform = from_transform.inverse() * to_transform;
        return true;
    }

    bool MoveItInterface::getLinkPose(const std::string &link_name,
                                      geometry_msgs::msg::Pose &pose) const
    {
        if (!isPSMValid(false))
            return false;

        planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
        const moveit::core::RobotState &state = scene->getCurrentState();

        if (!state.knowsFrameTransform(link_name))
        {
            RCLCPP_ERROR(node_->get_logger(), "Unknown link: %s", link_name.c_str());
            return false;
        }

        const Eigen::Isometry3d &transform = state.getGlobalLinkTransform(link_name);
        pose = tf2::toMsg(transform);

        return true;
    }

    bool MoveItInterface::getLinkPose(const std::string &link_name,
                                      Eigen::Vector3d &position,
                                      std::array<double, 4> &orientation) const
    {
        geometry_msgs::msg::Pose pose;
        if (!getLinkPose(link_name, pose))
        {
            return false;
        }
        
        // Convert to Eigen types
        position.x() = pose.position.x;
        position.y() = pose.position.y;
        position.z() = pose.position.z;
        orientation[0] = pose.orientation.x;
        orientation[1] = pose.orientation.y;
        orientation[2] = pose.orientation.z;
        orientation[3] = pose.orientation.w;
        
        return true;
    }

} // namespace husky_xarm6_mcr_nbv_planner
