#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("plan_linear_cart");

    // Parameters (override via launch if desired)
    const std::string group_name = node->declare_parameter<std::string>("group_name", "xarm6");
    const std::string eef_link = node->declare_parameter<std::string>("eef_link", "link_eef");
    const std::string world_frame = node->declare_parameter<std::string>("world_frame", "world");
    const double delta_x_world = node->declare_parameter<double>("delta_x_world", 0.3);  // +30 cm in world-x
    const double cart_eef_step = node->declare_parameter<double>("eef_step", 0.002);     // 2 mm
    const double cart_jump_thr = node->declare_parameter<double>("jump_threshold", 0.0); // disable
    const double min_fraction = node->declare_parameter<double>("min_fraction", 0.95);   // require 95%

    // TF buffer/listener
    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // MoveGroupInterface
    moveit::planning_interface::MoveGroupInterface mgi(node, group_name);
    mgi.setEndEffectorLink(eef_link);
    mgi.setPoseReferenceFrame(world_frame);
    mgi.setMaxVelocityScalingFactor(0.2);
    mgi.setMaxAccelerationScalingFactor(0.2);

    // --- 1) Move to your specified start pose in world ------------------------
    geometry_msgs::msg::PoseStamped start_ps;
    start_ps.header.frame_id = world_frame;
    start_ps.pose.position.x = -0.3;
    start_ps.pose.position.y = 0.097;
    start_ps.pose.position.z = 0.739;
    start_ps.pose.orientation.x = -0.707;
    start_ps.pose.orientation.y = 0.000;
    start_ps.pose.orientation.z = -0.000;
    start_ps.pose.orientation.w = 0.707;

    mgi.setStartStateToCurrentState();
    mgi.setPoseTarget(start_ps, eef_link);

    {
        moveit::planning_interface::MoveGroupInterface::Plan plan_to_start;
        auto ok = (mgi.plan(plan_to_start) == moveit::core::MoveItErrorCode::SUCCESS);
        if (!ok)
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to plan to the provided start pose.");
            rclcpp::shutdown();
            return 1;
        }
        auto exec_ok = (mgi.execute(plan_to_start) == moveit::core::MoveItErrorCode::SUCCESS);
        if (!exec_ok)
        {
            RCLCPP_ERROR(node->get_logger(), "Execution to start pose failed.");
            rclcpp::shutdown();
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "Reached start pose.");
    }

    // --- 2) Read current EEF pose from TF (world -> eef_link) -----------------
    geometry_msgs::msg::TransformStamped tf;
    try
    {
        tf = tf_buffer.lookupTransform(
            world_frame, eef_link, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR(node->get_logger(), "TF lookup failed: %s", ex.what());
        rclcpp::shutdown();
        return 1;
    }

    geometry_msgs::msg::Pose current_pose_world;
    current_pose_world.position.x = tf.transform.translation.x;
    current_pose_world.position.y = tf.transform.translation.y;
    current_pose_world.position.z = tf.transform.translation.z;
    current_pose_world.orientation = tf.transform.rotation;

    RCLCPP_INFO(node->get_logger(),
                "TF EEF pose (world): [%.3f, %.3f, %.3f]  q[x y z w]=[%.3f %.3f %.3f %.3f]",
                current_pose_world.position.x, current_pose_world.position.y, current_pose_world.position.z,
                current_pose_world.orientation.x, current_pose_world.orientation.y,
                current_pose_world.orientation.z, current_pose_world.orientation.w);

    // --- 3) Cartesian straight line +X(world) from the TF pose ----------------
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(current_pose_world);

    geometry_msgs::msg::Pose target_pose_world = current_pose_world;
    target_pose_world.position.x += delta_x_world; // move +X in world
    waypoints.push_back(target_pose_world);

    moveit_msgs::msg::RobotTrajectory traj;
    const double fraction = mgi.computeCartesianPath(
        waypoints, cart_eef_step, cart_jump_thr, traj, /*avoid_collisions*/ true);

    RCLCPP_INFO(node->get_logger(), "Cartesian path fraction: %.3f", fraction);
    if (fraction < min_fraction)
    {
        RCLCPP_ERROR(node->get_logger(),
                     "Only achieved %.1f%% of the path; not executing.", 100.0 * fraction);
        rclcpp::shutdown();
        return 1;
    }

    // ---- NEW: time-parameterize (slow it down) ---------------------------------
    robot_trajectory::RobotTrajectory rt(mgi.getRobotModel(), group_name);

    // Use the *current* state as reference for the RobotTrajectory
    rt.setRobotTrajectoryMsg(*mgi.getCurrentState(), traj);

    // Choose your scaling (smaller = slower)
    const double vel_scale = 0.1;   // 10% of max
    const double acc_scale = 0.1;   // 10% of max

    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    bool ok = iptp.computeTimeStamps(rt, vel_scale, acc_scale);
    if (!ok) {
        RCLCPP_ERROR(node->get_logger(), "Time parameterization failed.");
        rclcpp::shutdown();
        return 1;
    }

    // Convert back to a message for execution
    moveit_msgs::msg::RobotTrajectory timed_traj;
    rt.getRobotTrajectoryMsg(timed_traj);

    // Execute
    moveit::planning_interface::MoveGroupInterface::Plan cart_plan;
    cart_plan.trajectory_ = timed_traj;
    auto exec_ok = (mgi.execute(cart_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!exec_ok) {
        RCLCPP_ERROR(node->get_logger(), "Cartesian execution failed.");
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "Cartesian move completed.");
    rclcpp::shutdown();
    return 0;
}
