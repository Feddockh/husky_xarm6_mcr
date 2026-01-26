#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <chrono>
#include <queue>
#include <algorithm>
#include "husky_xarm6_mcr_nbv_planner/moveit_interface.hpp"
#include "husky_xarm6_mcr_nbv_planner/manipulation_workspace.hpp"
#include "husky_xarm6_mcr_nbv_planner/octomap_interface.hpp"
#include "husky_xarm6_mcr_nbv_planner/nbv_visualizer.hpp"
#include "husky_xarm6_mcr_nbv_planner/viewpoint_generation.hpp"
#include "husky_xarm6_mcr_nbv_planner/geometry_utils.hpp"
#include "husky_xarm6_mcr_nbv_planner/conversions.hpp"

using namespace husky_xarm6_mcr_nbv_planner;
using namespace husky_xarm6_mcr_nbv_planner::conversions;

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

    // Get all parameters
    std::string manipulator_group_name = node->get_parameter("manipulator_group_name").as_string();
    bool learn_workspace = node->get_parameter("learn_workspace").as_bool();
    std::string workspace_file = node->get_parameter("manipulation_workspace_file").as_string();
    int num_samples = node->get_parameter("num_samples").as_int();
    bool visualize = node->get_parameter("visualize").as_bool();
    std::string visualization_topic = node->get_parameter("visualization_topic").as_string();
    std::string octomap_topic = node->get_parameter("octomap_topic").as_string();
    int max_iterations = node->get_parameter("max_iterations").as_int();
    double min_information_gain = node->get_parameter("min_information_gain").as_double();
    double alpha_cost_weight = node->get_parameter("alpha_cost_weight").as_double();
    int num_viewpoints_per_frontier = node->get_parameter("num_viewpoints_per_frontier").as_int();
    double camera_horizontal_fov = node->get_parameter("camera_horizontal_fov").as_double();
    double camera_vertical_fov = node->get_parameter("camera_vertical_fov").as_double();
    int camera_width = node->get_parameter("camera_width").as_int();
    int camera_height = node->get_parameter("camera_height").as_int();
    double camera_max_range = node->get_parameter("camera_max_range").as_double();
    int num_camera_rays = node->get_parameter("num_camera_rays").as_int();
    std::string map_frame = node->get_parameter("map_frame").as_string();

    // Print NBV Planner Configuration
    RCLCPP_INFO(node->get_logger(), "\n=== NBV Volumetric Planner Configuration ===");
    RCLCPP_INFO(node->get_logger(), "Manipulator group: %s", manipulator_group_name.c_str());
    RCLCPP_INFO(node->get_logger(), "Learn workspace: %s", learn_workspace ? "true" : "false");
    RCLCPP_INFO(node->get_logger(), "Workspace samples: %d", num_samples);
    RCLCPP_INFO(node->get_logger(), "Visualization: %s", visualize ? "enabled" : "disabled");
    RCLCPP_INFO(node->get_logger(), "Octomap topic: %s", octomap_topic.c_str());
    RCLCPP_INFO(node->get_logger(), "Max iterations: %d", max_iterations);
    RCLCPP_INFO(node->get_logger(), "Min information gain: %.4f", min_information_gain);
    RCLCPP_INFO(node->get_logger(), "Alpha cost weight: %.4f", alpha_cost_weight);
    RCLCPP_INFO(node->get_logger(), "Viewpoints per frontier: %d", num_viewpoints_per_frontier);
    RCLCPP_INFO(node->get_logger(), "Camera H-FOV: %.1f rad (%.1f deg)", camera_horizontal_fov, camera_horizontal_fov * 180.0 / M_PI);
    RCLCPP_INFO(node->get_logger(), "Camera V-FOV: %.1f rad (%.1f deg)", camera_vertical_fov, camera_vertical_fov * 180.0 / M_PI);
    RCLCPP_INFO(node->get_logger(), "Camera resolution: %dx%d", camera_width, camera_height);
    RCLCPP_INFO(node->get_logger(), "Max sensor range: %.2f m", camera_max_range);
    RCLCPP_INFO(node->get_logger(), "Number of IG rays: %d", num_camera_rays);
    RCLCPP_INFO(node->get_logger(), "Map frame: %s", map_frame.c_str());
    RCLCPP_INFO(node->get_logger(), "============================================\n");

    // Initialize MoveIt interface
    auto interface = std::make_shared<MoveItInterface>(node, manipulator_group_name);
    interface->setPlanningPipelineId("ompl");
    interface->setPlannerId("RRTConnect");
    interface->setPlanningTime(1.0); // seconds
    interface->setNumPlanningAttempts(5);

    // Check
    RCLCPP_INFO(node->get_logger(), "Pipeline: %s", interface->getPlanningPipelineId().c_str());
    RCLCPP_INFO(node->get_logger(), "Planner ID: %s", interface->getPlannerId().c_str());
    RCLCPP_INFO(node->get_logger(), "Planning time: %.2f seconds", interface->getPlanningTime());
    RCLCPP_INFO(node->get_logger(), "Number of planning attempts: %d", interface->getNumPlanningAttempts());

    // Initialize visualizer if visualization is requested
    std::shared_ptr<NBVVisualizer> visualizer;
    if (visualize)
    {
        visualizer = std::make_shared<NBVVisualizer>(node, map_frame, visualization_topic);
        RCLCPP_INFO(node->get_logger(), "Visualizer initialized on topic: %s", visualization_topic.c_str());
    }

    // Define the initial joint angles and move to initial viewpoint
    std::vector<double> init_joint_angles_deg = {0.0, -45.0, -45.0, 0.0, 0.0, 90.0};
    std::vector<double> init_joint_angles_rad = geometry_utils::deg2Rad(init_joint_angles_deg);
    RCLCPP_INFO(node->get_logger(), "\nMoving to initial joint configuration...");
    if (!interface->planAndExecute(init_joint_angles_rad))
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to move to initial joint configuration");
        rclcpp::shutdown();
        return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Moved to initial joint configuration successfully");

    // Get the pose of the optical frame of the camera and display it
    std::string camera_optical_link = node->get_parameter("camera_optical_link").as_string();
    Eigen::Vector3d init_cam_position;
    std::array<double, 4> init_cam_orientation;
    if (!interface->getLinkPose(camera_optical_link, init_cam_position, init_cam_orientation))
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to get camera link pose");
        rclcpp::shutdown();
        return 1;
    }
    if (visualizer)
    {
        visualizer->publishCoordinate(init_cam_position, init_cam_orientation, 0.15, 0.01, 1.0f, "initial_camera_pose");
        RCLCPP_INFO(node->get_logger(), "Initial camera pose published for visualization");
    }

    // Step 1: Learn or load the manipulation workspace
    RCLCPP_INFO(node->get_logger(), "\n=== Step 1: Manipulation Workspace ===");
    RCLCPP_INFO(node->get_logger(), "Initializing Manipulation Workspace...");
    auto workspace = std::make_shared<ManipulationWorkspace>(interface, visualizer);
    if (learn_workspace)
    {
        RCLCPP_INFO(node->get_logger(), "Learning workspace with %d samples...", num_samples);
        if (!workspace->learnWorkspace(num_samples, visualize))
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to learn workspace");
            rclcpp::shutdown();
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "Learned %zu voxels, saving to: %s",
                    workspace->getNumReachableVoxels(), workspace_file.c_str());
        workspace->saveWorkspaceToFile(workspace_file);
    }
    else
    {
        RCLCPP_INFO(node->get_logger(), "Loading workspace from: %s", workspace_file.c_str());
        if (!workspace->loadWorkspaceFromFile(workspace_file))
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to load. Set learn_workspace:=true");
            rclcpp::shutdown();
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "Loaded %zu voxels", workspace->getNumReachableVoxels());
    }

    // Step 2: Receive the OctoMap
    RCLCPP_INFO(node->get_logger(), "\n=== Step 2: Receive OctoMap ===");
    RCLCPP_INFO(node->get_logger(), "Initializing OctoMap Interface...");
    auto octomap_interface = std::make_shared<OctoMapInterface>(node, octomap_topic, true);
    RCLCPP_INFO(node->get_logger(), "Waiting for octomap to be published on %s...", octomap_topic.c_str());
    rclcpp::Time start_time = node->now();
    rclcpp::Rate spin_rate(10); // 10 Hz
    while (rclcpp::ok() && !octomap_interface->isTreeAvailable() && (node->now() - start_time).seconds() < 30.0)
    {
        rclcpp::spin_some(node);
        spin_rate.sleep();
    }
    if (!octomap_interface->isTreeAvailable())
    {
        RCLCPP_ERROR(node->get_logger(), "Octomap not available after 30 seconds. Exiting.");
        rclcpp::shutdown();
        return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Octomap received successfully");

    // // DEBUG: Delay
    // rclcpp::Rate rate(10); // 10 Hz
    // for (int i = 0; i < 100; ++i) { rclcpp::spin_some(node); rate.sleep(); } // ~10 second pause

    // Main NBV Planning Loop
    for (int iter = 0; iter < max_iterations; ++iter)
    {
        RCLCPP_INFO(node->get_logger(), "\n********** NBV Planning Iteration %d **********", iter + 1);

        // Step 3: Compute frontiers
        RCLCPP_INFO(node->get_logger(), "\n=== Step 3: Compute Frontiers ===");
        int min_unknown_neighbors = 1; // Minimum unknown neighbors for a voxel to be considered a frontier
        bool use_frontier_bbox = true; // Use bounding box to limit frontier search
        std::vector<octomap::point3d> frontiers = octomap_interface->findFrontiers(min_unknown_neighbors, use_frontier_bbox);
        RCLCPP_INFO(node->get_logger(), "Found %zu frontiers", frontiers.size());
        std::vector<Cluster> frontier_clusters;
        if (frontiers.empty())
        {
            RCLCPP_WARN(node->get_logger(), "No frontiers found - exploration may be complete or bbox too restrictive");
            break;
        }
        else
        {
            // Cluster the frontiers
            int n_clusters = std::max(1, (int)frontiers.size() / 100); // Roughly 100 frontiers per cluster
            int max_kmeans_iters = 50;
            double convergence_tol = 1e-4;
            frontier_clusters = octomap_interface->kmeansCluster(frontiers, n_clusters, max_kmeans_iters, convergence_tol);
            RCLCPP_INFO(node->get_logger(), "Clustered frontiers into %zu groups", frontier_clusters.size());
            if (visualizer)
            {
                double octomap_resolution;
                octomap_interface->getResolution(octomap_resolution);
                visualizer->publishClusteredFrontiers(frontier_clusters, octomap_resolution, false, 0.8f, "frontier_clusters");
                RCLCPP_INFO(node->get_logger(), "Frontier clusters visualized");
            }
        }

        // Step 4: Generate the viewpoint candidates
        RCLCPP_INFO(node->get_logger(), "\n=== Step 4: Generate Viewpoint Candidates ===");
        // Generate viewpoints on a plane of spherical caps in front of the current camera position
        std::vector<Viewpoint> spherical_planar_viewpoints = generatePlanarSphericalCapCandidates(init_cam_position, init_cam_orientation, 0.4, 0.2, geometry_utils::deg2Rad(50.0), geometry_utils::deg2Rad(25.0));
        RCLCPP_INFO(node->get_logger(), "Generated %zu viewpoints on planar spherical cap", spherical_planar_viewpoints.size());
        // Generate viewpoints around each frontier cluster
        std::vector<Viewpoint> frontier_viewpoints;
        double best_range = 0.5;      // Optimal viewing distance for frontiers
        double range_tolerance = 0.1; // Allow slight variation in range
        for (const auto &cluster : frontier_clusters)
        {
            Eigen::Vector3d cluster_center(cluster.center.x(), cluster.center.y(), cluster.center.z());
            std::vector<Viewpoint> cluster_viewpoints = sampleViewsFromHemisphere(
                cluster_center,
                init_cam_orientation,
                best_range,
                best_range + range_tolerance,
                num_viewpoints_per_frontier,
                false,      // use_positive_z = false (sample below as well)
                M_PI / 3.0, // z_bias_sigma = pi/3 (bias toward horizontal views)
                0.05,       // min_distance between viewpoints
                1000        // max_attempts
            );
            frontier_viewpoints.insert(frontier_viewpoints.end(), cluster_viewpoints.begin(), cluster_viewpoints.end());
        }
        RCLCPP_INFO(node->get_logger(), "Generated %zu viewpoints around %zu frontier clusters",
                    frontier_viewpoints.size(), frontier_clusters.size());
        // Combine all viewpoints
        std::vector<Viewpoint> total_viewpoints;
        total_viewpoints.insert(total_viewpoints.end(), spherical_planar_viewpoints.begin(), spherical_planar_viewpoints.end());
        total_viewpoints.insert(total_viewpoints.end(), frontier_viewpoints.begin(), frontier_viewpoints.end());

        // Step 5: Filter viewpoints by workspace reachability
        RCLCPP_INFO(node->get_logger(), "\n=== Step 5: Filter Viewpoints by Workspace Reachability ===");
        std::vector<Viewpoint> reachable_viewpoints;
        for (const auto &vp : total_viewpoints)
        {
            if (workspace->isPoseReachable(eigenToPose(vp.position, vp.orientation)))
            {
                reachable_viewpoints.push_back(vp);
            }
        }
        RCLCPP_INFO(node->get_logger(), "%zu out of %zu viewpoints are reachable",
                    reachable_viewpoints.size(), total_viewpoints.size());
        if (visualizer && !reachable_viewpoints.empty())
        {
            RCLCPP_INFO(node->get_logger(), "Visualizing reachable viewpoints...");
            // Convert all viewpoints to poses for batch visualization
            std::vector<geometry_msgs::msg::Pose> viewpoint_poses;
            for (const auto &vp : reachable_viewpoints)
            {
                viewpoint_poses.push_back(eigenToPose(vp.position, vp.orientation));
            }
            visualizer->publishCoordinates(viewpoint_poses, 0.1, 0.005, 0.5f, "reachable_viewpoints");
            RCLCPP_INFO(node->get_logger(), "Visualization complete!");
        }

        // Step 6: Compute information gain for each viewpoint
        RCLCPP_INFO(node->get_logger(), "\n=== Step 6: Compute Information Gain for Each Reachable Viewpoint ===");
        double octomap_resolution;
        octomap_interface->getResolution(octomap_resolution);
        double average_information_gain = 0.0;
        for (size_t i = 0; i < reachable_viewpoints.size(); ++i)
        {
            double info_gain = computeInformationGain(
                reachable_viewpoints[i],
                octomap_interface,
                geometry_utils::rad2Deg(camera_horizontal_fov), // Expects degrees
                camera_width,
                camera_height,
                camera_max_range,
                octomap_resolution,
                num_camera_rays,
                true, // use_bbox
                node->get_logger(),
                nullptr); // DEBUG: visualize
            // RCLCPP_INFO(node->get_logger(), "Viewpoint %zu: Information Gain = %.4f", i + 1, info_gain);
            reachable_viewpoints[i].information_gain = info_gain;
            average_information_gain += info_gain;
        }
        average_information_gain /= static_cast<double>(reachable_viewpoints.size());
        RCLCPP_INFO(node->get_logger(), "Average Information Gain across reachable viewpoints: %.4f", average_information_gain);

        // Step 7: Compute utility for each viewpoint
        RCLCPP_INFO(node->get_logger(), "\n=== Step 7: Compute Utility for Each Viewpoint ===");
        Eigen::Vector3d current_cam_position;
        std::array<double, 4> current_cam_orientation;
        interface->getLinkPose(camera_optical_link, current_cam_position, current_cam_orientation);
        double max_cost, min_cost = 0.0;
        for (size_t i = 0; i < reachable_viewpoints.size(); ++i)
        {
            double distance = (reachable_viewpoints[i].position - current_cam_position).norm();
            double cost = distance; // Simple cost based on distance
            double utility = reachable_viewpoints[i].information_gain - alpha_cost_weight * cost;
            // RCLCPP_INFO(node->get_logger(), "Viewpoint %zu: Cost = %.4f, Utility = %.4f", i + 1, cost, utility);
            reachable_viewpoints[i].cost = cost;
            reachable_viewpoints[i].utility = utility;
            if (i == 0 || cost > max_cost)
                max_cost = cost;
            if (i == 0 || cost < min_cost)
                min_cost = cost;
        }
        RCLCPP_INFO(node->get_logger(), "Cost range across viewpoints: [%.4f, %.4f]", min_cost, max_cost);

        // Step 8: Order viewpoints by utility and select the best one with valid IK
        RCLCPP_INFO(node->get_logger(), "\n=== Step 8: Select the Best Viewpoint with Valid IK ===");
        std::vector<double> current_joint_state; // Use current joint angles as IK seed
        if (!interface->getCurrentJointAngles(current_joint_state))
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to get current joint state for IK seed");
            rclcpp::shutdown();
            return 1;
        }
        // Create a max-heap (priority queue) of viewpoints based on utility
        std::priority_queue<Viewpoint> viewpoint_heap;
        for (const auto &vp : reachable_viewpoints)
            viewpoint_heap.push(vp);
        // Try viewpoints in order of utility until we find one with valid IK
        Viewpoint best_viewpoint;
        std::vector<double> best_joint_angles;
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool found_valid_viewpoint = false;
        while (!viewpoint_heap.empty() && !found_valid_viewpoint)
        {
            best_viewpoint = viewpoint_heap.top();
            viewpoint_heap.pop();
            if (best_viewpoint.information_gain < min_information_gain)
            {
                RCLCPP_WARN(node->get_logger(), "No viewpoints left with information gain above threshold %.4f",
                            min_information_gain);
                break;
            }
            RCLCPP_INFO(node->get_logger(), "Trying viewpoint with utility %.4f, IG %.4f, cost %.4f",
                        best_viewpoint.utility, best_viewpoint.information_gain, best_viewpoint.cost);
            // Convert viewpoint camera pose to end-effector pose
            geometry_msgs::msg::Pose cam_pose = eigenToPose(best_viewpoint.position, best_viewpoint.orientation);
            geometry_msgs::msg::Pose target_ee_pose;
            if (!interface->cameraPoseToEEPose(cam_pose, camera_optical_link, target_ee_pose))
            {
                RCLCPP_DEBUG(node->get_logger(), "  Failed to convert camera pose to EE pose, trying next viewpoint");
                continue;
            }
            // Compute IK using current joint state as seed
            double timeout = 0.01; // seconds
            int attempts = 5;
            best_joint_angles = interface->computeIK(current_joint_state, target_ee_pose, timeout, attempts);
            if (!best_joint_angles.empty())
            {
                // Check if the IK solution is collision-free
                if (interface->isStateValid(best_joint_angles))
                    RCLCPP_INFO(node->get_logger(), "Found valid viewpoint with collision-free IK solution!");
                else
                {
                    RCLCPP_INFO(node->get_logger(), "  IK solution is in collision, trying next viewpoint...");
                    continue;
                }
            }
            else
            {
                RCLCPP_INFO(node->get_logger(), "  No IK solution found for this viewpoint, trying next one...");
                continue;
            }
            // Compute path to ensure reachability
            interface->planToJointGoal(best_joint_angles, plan);
            if (!plan.trajectory_.joint_trajectory.points.empty())
            {
                found_valid_viewpoint = true;
                RCLCPP_INFO(node->get_logger(), "Found valid viewpoint with reachable path!");
            }
            else
            {
                RCLCPP_INFO(node->get_logger(), "  No reachable path to IK solution, trying next viewpoint...");
                continue;
            }
        }
        // Report the best viewpoint found
        if (!found_valid_viewpoint)
        {
            RCLCPP_WARN(node->get_logger(), "No valid viewpoint with path to IK solution found!");
            break; // Exit the main loop if no valid viewpoint found
        }
        else
        {
            RCLCPP_INFO(node->get_logger(), "Best Valid Viewpoint:");
            RCLCPP_INFO(node->get_logger(), "  Position: [%.3f, %.3f, %.3f]",
                        best_viewpoint.position.x(),
                        best_viewpoint.position.y(),
                        best_viewpoint.position.z());
            RCLCPP_INFO(node->get_logger(), "  Information Gain: %.4f", best_viewpoint.information_gain);
            RCLCPP_INFO(node->get_logger(), "  Cost: %.4f", best_viewpoint.cost);
            RCLCPP_INFO(node->get_logger(), "  Utility: %.4f", best_viewpoint.utility);

            if (visualizer)
            {
                visualizer->publishCoordinate(eigenToPose(best_viewpoint.position, best_viewpoint.orientation),
                                              0.15, 0.01, 1.0f, "best_viewpoint");
                RCLCPP_INFO(node->get_logger(), "Best viewpoint published for visualization");
            }
        }

        // Step 9: Move to the best viewpoint
        RCLCPP_INFO(node->get_logger(), "\n=== Step 9: Move to the Best Viewpoint ===");
        if (found_valid_viewpoint)
        {
            // Record the octomap timestamp before motion
            rclcpp::Time octomap_time_before_motion = octomap_interface->getLastUpdateTime();

            RCLCPP_INFO(node->get_logger(), "Executing motion to best viewpoint...");
            if (interface->execute(plan))
            {
                RCLCPP_INFO(node->get_logger(), "Successfully moved to best viewpoint!");

                // Wait for both motion completion and octomap update
                RCLCPP_INFO(node->get_logger(), "Waiting for manipulator to reach goal and octomap to update...");
                rclcpp::Rate spin_rate(10); // 10 Hz to process callbacks
                bool motion_complete = false;
                bool octomap_updated = false;
                while (rclcpp::ok() && (!motion_complete || !octomap_updated))
                {
                    rclcpp::spin_some(node); // Process callbacks
                    // Check if manipulator has reached the goal
                    if (!motion_complete)
                    {
                        std::vector<double> current_joints;
                        if (interface->getCurrentJointAngles(current_joints))
                        {
                            // Check if current joint angles match target (within tolerance)
                            double joint_error = 0.0;
                            for (size_t i = 0; i < current_joints.size() && i < best_joint_angles.size(); ++i)
                            {
                                joint_error += std::abs(current_joints[i] - best_joint_angles[i]);
                            }
                            if (joint_error < 0.01) // 0.01 radian tolerance (~0.57 degrees total)
                            {
                                motion_complete = true;
                                RCLCPP_INFO(node->get_logger(), "Manipulator reached goal position");
                            }
                        }
                    }
                    // Check if octomap has been updated
                    if (!octomap_updated)
                    {
                        rclcpp::Time current_octomap_time = octomap_interface->getLastUpdateTime();
                        if (current_octomap_time != octomap_time_before_motion)
                        {
                            octomap_updated = true;
                            RCLCPP_INFO(node->get_logger(), "Octomap has been updated with new sensor data");
                        }
                    }
                    spin_rate.sleep();
                }
                RCLCPP_INFO(node->get_logger(), "Motion complete and octomap updated - ready for next iteration");
            }
            else
                RCLCPP_ERROR(node->get_logger(), "Failed to execute motion to best viewpoint!");
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "No viewpoint found to move to, ending NBV planning");
            break;
        }

        // Clear all markers and spin
        if (visualizer)
        {
            visualizer->clearAllMarkers();
            RCLCPP_INFO(node->get_logger(), "Cleared all visualization markers");
        }
    }

    RCLCPP_INFO(node->get_logger(), "Press Ctrl+C to exit.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
