#include <chrono>
#include <thread>
#include <vector>
#include <map>
#include <string>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

static inline double deg2rad(double d) { return d * M_PI / 180.0; }

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("joint_waypoints_xarm6");

    // Planning group for xArm6 (override with a launch param if needed)
    const std::string group_name = node->declare_parameter<std::string>("group_name", "xarm6");
    const double vel_scale = node->declare_parameter<double>("velocity_scaling", 0.3);
    const double acc_scale = node->declare_parameter<double>("acceleration_scaling", 0.3);
    const double planning_time = node->declare_parameter<double>("planning_time", 10.0);
    const int attempts = node->declare_parameter<int>("planning_attempts", 10);
    const double dwell_s = node->declare_parameter<double>("dwell_seconds", 0.5);

    moveit::planning_interface::MoveGroupInterface arm(node, group_name);
    arm.setMaxVelocityScalingFactor(vel_scale);
    arm.setMaxAccelerationScalingFactor(acc_scale);
    arm.setPlanningTime(planning_time);
    arm.setNumPlanningAttempts(attempts);
    arm.startStateMonitor();

    // Waypoints (joint1..joint6) in DEGREES, exactly as provided
    // Order is joint1, joint2, joint3, joint4, joint5, joint6
    const std::vector<std::array<double, 6>> waypoints_deg = {
        /* Top Left     */ {0.0, -50.2, -135.0, 90.0, 90.0, 95.1},
        /* Top Center   */ {0.0, -44.8, -102.2, 90.0, 90.0, 56.8},
        /* Top Right    */ {0.0, 0.5, -135.0, 90.0, 90.0, 44.4},
        /* Center Right */ {0.0, -29.7, -71.8, 90.0, 90.0, 11.4},
        /* Center Center*/ {0.0, -79.6, -50.6, 90.0, 90.0, 40.0},
        /* Center Left  */ {0.0, -99.1, -71.9, 90.0, 90.0, 80.8},
        /* Bottom Left  */ {0.0, -109.3, -57.3, -90.0, -90.0, -103.8},
        /* Bottom Center*/ {0.0, -73.6, -21.6, -90.0, -90.0, -177.0},
        //   /* Bottom Right */ {  0.0,   -5.1,  -57.2,  -90.0,  -90.0, -150.1}
        /* Bottom Right */ {0.0, -36.1, -39.6, 90.0, 90.0, -14.7}};

    RCLCPP_INFO(node->get_logger(), "Starting loop through %zu joint waypoints for group '%s'...",
                waypoints_deg.size(), group_name.c_str());

    while (rclcpp::ok())
    {
        for (size_t i = 0; rclcpp::ok() && i < waypoints_deg.size(); ++i)
        {
            // Build a name->value map to avoid relying on joint vector ordering
            std::map<std::string, double> target_rad = {
                {"joint1", deg2rad(waypoints_deg[i][0])},
                {"joint2", deg2rad(waypoints_deg[i][1])},
                {"joint3", deg2rad(waypoints_deg[i][2])},
                {"joint4", deg2rad(waypoints_deg[i][3])},
                {"joint5", deg2rad(waypoints_deg[i][4])},
                {"joint6", deg2rad(waypoints_deg[i][5])},
            };

            RCLCPP_INFO(node->get_logger(),
                        "Waypoint %zu: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f] deg",
                        i + 1,
                        waypoints_deg[i][0], waypoints_deg[i][1], waypoints_deg[i][2],
                        waypoints_deg[i][3], waypoints_deg[i][4], waypoints_deg[i][5]);

            arm.setStartStateToCurrentState();
            arm.setJointValueTarget(target_rad);

            // Simple: plan then execute, with a small retry loop
            constexpr int retries = 3;
            bool done = false;
            for (int attempt = 1; attempt <= retries && rclcpp::ok(); ++attempt)
            {
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                auto ok_plan = (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                if (!ok_plan)
                {
                    RCLCPP_WARN(node->get_logger(), "plan() failed (attempt %d/%d).", attempt, retries);
                    continue;
                }
                auto ok_exec = (arm.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                if (!ok_exec)
                {
                    RCLCPP_WARN(node->get_logger(), "execute() failed (attempt %d/%d).", attempt, retries);
                    continue;
                }
                done = true;
                break;
            }

            if (!done)
            {
                RCLCPP_ERROR(node->get_logger(), "Failed to reach waypoint %zu after retries. Exiting.", i + 1);
                rclcpp::shutdown();
                return 1;
            }

            std::this_thread::sleep_for(std::chrono::duration<double>(dwell_s));
        }
    }

    rclcpp::shutdown();
    return 0;
}
