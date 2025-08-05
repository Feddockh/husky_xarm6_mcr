// This works pretty poorly because MoveIt!'s Cartesian path planner is poor and not constrainted

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

// -----------------------------------------------------------------------------
// TF helper: world -> eef_link pose
// -----------------------------------------------------------------------------
static geometry_msgs::msg::PoseStamped lookup_eef_pose_tf(
    const rclcpp::Node::SharedPtr &node,
    tf2_ros::Buffer &tf_buffer,
    const std::string &world_frame,
    const std::string &eef_link,
    double timeout_sec = 1.0)
{
    geometry_msgs::msg::TransformStamped tf;
    tf = tf_buffer.lookupTransform(
        world_frame, eef_link, rclcpp::Time(0),
        rclcpp::Duration::from_seconds(timeout_sec));

    geometry_msgs::msg::PoseStamped ps;
    ps.header = tf.header;
    ps.pose.position.x = tf.transform.translation.x;
    ps.pose.position.y = tf.transform.translation.y;
    ps.pose.position.z = tf.transform.translation.z;
    ps.pose.orientation = tf.transform.rotation;
    return ps;
}

// -----------------------------------------------------------------------------
// OMPL plan + exec to a pose target (world frame)
// -----------------------------------------------------------------------------
static bool plan_to_pose_ompl(
    rclcpp::Node::SharedPtr node,
    moveit::planning_interface::MoveGroupInterface &mgi,
    const geometry_msgs::msg::PoseStamped &goal_ps,
    const std::string &eef_link)
{
    mgi.setStartStateToCurrentState();
    mgi.setPoseTarget(goal_ps, eef_link);

    moveit::planning_interface::MoveGroupInterface::Plan plan_to_start;
    auto ok_plan = (mgi.plan(plan_to_start) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!ok_plan)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to plan to start/goal pose.");
        return false;
    }
    auto ok_exec = (mgi.execute(plan_to_start) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!ok_exec)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to execute to start/goal pose.");
        return false;
    }
    return true;
}

// -----------------------------------------------------------------------------
// Compute & execute a straight Cartesian delta in WORLD (x,y,z).
// - Robustness: retries with decreasing eef_step
// - Time-parameterizes trajectory (IPTP) using vel/acc scaling
// -----------------------------------------------------------------------------
static bool cartesian_delta_exec(
    rclcpp::Node::SharedPtr node,
    tf2_ros::Buffer &tf_buffer,
    moveit::planning_interface::MoveGroupInterface &mgi,
    const std::string &world_frame,
    const std::string &eef_link,
    double dx, double dy, double dz,
    double min_fraction,
    double vel_scale, double acc_scale,
    double base_eef_step = 0.002, // 2mm base
    double jump_thr = 0.0,        // disable jump check
    bool avoid_collisions = true)
{
    // Current EEF pose from TF
    geometry_msgs::msg::PoseStamped cur;
    try
    {
        cur = lookup_eef_pose_tf(node, tf_buffer, world_frame, eef_link, 1.0);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR(node->get_logger(), "TF lookup failed: %s", ex.what());
        return false;
    }

    // Waypoints: current -> offset
    std::vector<geometry_msgs::msg::Pose> wps;
    wps.push_back(cur.pose);

    geometry_msgs::msg::Pose tgt = cur.pose;
    tgt.position.x += dx;
    tgt.position.y += dy;
    tgt.position.z += dz;
    wps.push_back(tgt);

    // Retry with smaller eef_step
    moveit_msgs::msg::RobotTrajectory best;
    double best_fraction = 0.0;

    for (double step : {base_eef_step, 0.0015, 0.0010, 0.0007, 0.0005})
    {
        moveit_msgs::msg::RobotTrajectory temp;
        double frac = mgi.computeCartesianPath(wps, step, jump_thr, temp, avoid_collisions);
        RCLCPP_INFO(node->get_logger(), "Cartesian fraction (step=%.4f): %.3f", step, frac);
        if (frac > best_fraction)
        {
            best_fraction = frac;
            best = temp;
        }
        if (frac >= 0.999)
            break;
    }

    if (best_fraction < min_fraction)
    {
        RCLCPP_ERROR(node->get_logger(), "Path fraction %.1f%% < required %.1f%%; abort.",
                     100.0 * best_fraction, 100.0 * min_fraction);
        return false;
    }

    // Time-parameterize
    robot_trajectory::RobotTrajectory rt(mgi.getRobotModel(), mgi.getName());
    rt.setRobotTrajectoryMsg(*mgi.getCurrentState(), best);

    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    if (!iptp.computeTimeStamps(rt, vel_scale, acc_scale))
    {
        RCLCPP_WARN(node->get_logger(), "Time parameterization failed; executing unretimed.");
    }

    moveit_msgs::msg::RobotTrajectory timed;
    rt.getRobotTrajectoryMsg(timed);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = timed;
    auto ok_exec = (mgi.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!ok_exec)
    {
        RCLCPP_ERROR(node->get_logger(), "Cartesian execution failed.");
        return false;
    }
    return true;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("plan_square_cart");

    // ---------------- Parameters ----------------
    const std::string group_name = node->declare_parameter<std::string>("group_name", "xarm6");
    const std::string eef_link = node->declare_parameter<std::string>("eef_link", "link_eef");
    const std::string world_frame = node->declare_parameter<std::string>("world_frame", "world");

    // Start pose (same as your earlier code; change if needed)
    geometry_msgs::msg::PoseStamped start_ps;
    start_ps.header.frame_id = world_frame;
    start_ps.pose.position.x = -0.3;
    start_ps.pose.position.y = 0.097;
    start_ps.pose.position.z = 0.739;
    start_ps.pose.orientation.x = -0.707;
    start_ps.pose.orientation.y = 0.0;
    start_ps.pose.orientation.z = 0.0;
    start_ps.pose.orientation.w = 0.707;

    // Raster parameters
    const double dx_cell = node->declare_parameter<double>("dx_cell", 0.3); // 30 cm steps in X
    const double dz_row = node->declare_parameter<double>("dz_row", 0.1);   // 0 cm down/up
    const double min_fraction = node->declare_parameter<double>("min_fraction", 0.95);
    const double eef_step = node->declare_parameter<double>("eef_step", 0.002);
    const double vel_scale = node->declare_parameter<double>("vel_scale", 0.10);
    const double acc_scale = node->declare_parameter<double>("acc_scale", 0.10);
    const int loops = node->declare_parameter<int>("loops", -1); // -1 = infinite

    // MoveIt group
    moveit::planning_interface::MoveGroupInterface mgi(node, group_name);
    mgi.setEndEffectorLink(eef_link);
    mgi.setPoseReferenceFrame(world_frame);
    mgi.setMaxVelocityScalingFactor(vel_scale);
    mgi.setMaxAccelerationScalingFactor(acc_scale);
    mgi.setPlanningTime(10.0);
    mgi.setNumPlanningAttempts(10);

    // TF
    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // 1) Move to starting pose with OMPL
    if (!plan_to_pose_ompl(node, mgi, start_ps, eef_link))
    {
        rclcpp::shutdown();
        return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Reached start pose.");

    // 2) Build one cycle of the requested motion (deltas in world)
    // Sequence:
    // +0.3, +0.3, -0.2z, -0.3, -0.3, -0.2z, +0.3, +0.3, -0.6, +0.4z
    struct Delta
    {
        double x, y, z;
    };
    const std::vector<Delta> cycle = {
        {+dx_cell, 0, 0}, {+dx_cell, 0, 0}, {0, 0, -dz_row}, {-dx_cell, 0, 0}, {-dx_cell, 0, 0}, {0, 0, -dz_row}, {+dx_cell, 0, 0}, {+dx_cell, 0, 0}, {-2.0 * dx_cell, 0, 0}, {0, 0, +2.0 * dz_row}};

    // 3) Loop execution
    int iter = 0;
    while (rclcpp::ok())
    {
        RCLCPP_INFO(node->get_logger(), "Starting cycle %d", iter + 1);
        for (const auto &d : cycle)
        {
            if (!cartesian_delta_exec(node, tf_buffer, mgi, world_frame, eef_link,
                                      d.x, d.y, d.z,
                                      min_fraction, vel_scale, acc_scale, eef_step, 0.0, true))
            {
                RCLCPP_ERROR(node->get_logger(), "Segment failed; stopping.");
                rclcpp::shutdown();
                return 1;
            }
        }

        ++iter;
        if (loops >= 0 && iter >= loops)
        {
            RCLCPP_INFO(node->get_logger(), "Completed %d loop(s).", iter);
            break;
        }
    }

    RCLCPP_INFO(node->get_logger(), "Done.");
    rclcpp::shutdown();
    return 0;
}
