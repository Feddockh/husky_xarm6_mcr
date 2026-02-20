/**
 * @file nbv_volumetric_planner_demo.cpp
 * @brief Next-Best-View volumetric planner demo for manipulation workspace
 */

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <chrono>
#include <queue>
#include <algorithm>
#include <optional>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include "husky_xarm6_mcr_nbv_planner/moveit_interface.hpp"
#include "husky_xarm6_mcr_nbv_planner/manipulation_workspace.hpp"
#include "husky_xarm6_mcr_nbv_planner/octomap_interface.hpp"
#include "husky_xarm6_mcr_nbv_planner/nbv_visualizer.hpp"
#include "husky_xarm6_mcr_nbv_planner/viewpoint_generation.hpp"
#include "husky_xarm6_mcr_nbv_planner/geometry_utils.hpp"
#include "husky_xarm6_mcr_nbv_planner/conversions.hpp"
#include "husky_xarm6_mcr_nbv_planner/nbv_planner_utils.hpp"
#include "husky_xarm6_mcr_nbv_planner/nbv_types.hpp"

using namespace husky_xarm6_mcr_nbv_planner;
using namespace husky_xarm6_mcr_nbv_planner::conversions;


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "moveit_interface_node",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Wait for sim time if needed
    if (node->get_parameter("use_sim_time").as_bool()) {
        waitForSimClock(node);
    }

    // Load configuration
    auto config = loadConfiguration(node);
    printConfiguration(config, node->get_logger());

    // Create trigger clients and stop any ongoing video capture
    auto trigger_clients = createTriggerClients(node);
    stopVideoCapture(node, trigger_clients, node->get_logger());

    // Initialize MoveIt interface
    auto moveit_interface = setupMoveItInterface(node, config);
    RCLCPP_DEBUG(node->get_logger(), "MoveIt Interface initialized for group: %s with base link: %s",
                config.manipulator_group_name.c_str(), moveit_interface->getPoseReferenceFrame().c_str());

    // Initialize visualizer if requested
    std::shared_ptr<NBVVisualizer> visualizer;
    if (config.visualize) {
        visualizer = std::make_shared<NBVVisualizer>(node, config.map_frame, config.visualization_topic);
    }

    // Move to initial joint configuration
    RCLCPP_DEBUG(node->get_logger(), "\nMoving to initial joint configuration...");
    if (!moveit_interface->planAndExecute(config.init_joint_angles_rad))
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to move to initial joint configuration");
        rclcpp::shutdown();
        return 1;
    }

    // Get initial camera pose wrt moveit base link
    Eigen::Vector3d init_cam_position;
    std::array<double, 4> init_cam_orientation;
    if (!moveit_interface->getLinkPose(config.camera_optical_link, init_cam_position, init_cam_orientation))
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to get camera link pose");
        rclcpp::shutdown();
        return 1;
    }

    // Set orientation constraints (±90 degrees tolerance)
    moveit_interface->setOrientationConstraints(
        config.camera_optical_link,
        arrayToQuaternion(init_cam_orientation),
        M_PI/2, M_PI/2, M_PI);

    // Setup workspace with moveit interface
    auto manip_workspace = setupWorkspace(moveit_interface, visualizer, config, node->get_logger());
    if (!manip_workspace)
    {
        rclcpp::shutdown();
        return 1;
    }

    // Initialize TF2 buffer and listener
    RCLCPP_DEBUG(node->get_logger(), "\nInitializing TF2 Buffer and Listener");
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // Initialize octomap interface
    RCLCPP_DEBUG(node->get_logger(), "\nInitializing OctoMap Interface");
    auto octomap_interface = std::make_shared<OctoMapInterface>(node, config.octomap_topic, true);

    // Handle camera triggering based on capture_type
    if (config.capture_type == "continuous")
    {
        startContinuousCapture(node, trigger_clients, node->get_logger());
    }
    else if (config.capture_type == "triggered")
    {
        if (!trigger_clients.send_trigger->wait_for_service(std::chrono::seconds(5)))
            RCLCPP_WARN(node->get_logger(), "send_trigger service not available, waiting for octomap anyway...");
    }

    // Wait for initial octomap
    waitForOctomap(node, octomap_interface, trigger_clients, config, node->get_logger());

    // Initialize evaluation if enabled
    std::vector<EvaluationMetrics> all_metrics;
    double initial_time = 0.0;
    if (config.enable_evaluation && octomap_interface->isSemanticTree())
    {
        if (!octomap_interface->loadGroundTruthSemantics(config.gt_points_file))
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to load ground truth file: %s", config.gt_points_file.c_str());
            rclcpp::shutdown();
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "Ground truth loaded successfully");
        // Store initial time for relative time calculations
        initial_time = node->now().seconds();
        // Perform initial evaluation
        EvaluationMetrics eval_metrics;
        eval_metrics.time = 0.0;  // Initial evaluation at t=0
        auto latest_clusters = octomap_interface->clusterSemanticVoxels(false);
        auto match_result = octomap_interface->matchClustersToGroundTruth(latest_clusters, config.eval_threshold_radius, false);
        eval_metrics.class_metrics = octomap_interface->evaluateMatchResults(match_result, false);
        auto [occupied, free] = octomap_interface->getVoxelCounts();
        eval_metrics.occupied_voxels = occupied;
        eval_metrics.free_voxels = free;
        eval_metrics.bbox_coverage = octomap_interface->calculateCoverage();
        all_metrics.push_back(eval_metrics);
        if (visualizer)
        {
            visualizer->publishMatchResults(match_result, config.eval_threshold_radius * 2, 0.8f);
        }
        visualizer->plotAllMetrics(all_metrics, config.metrics_plots_dir);
        visualizer->logAllMetricsToCSV(all_metrics, config.metrics_data_dir);
    }
    else
        RCLCPP_WARN(node->get_logger(), "Evaluation disabled or octomap is not semantic, skipping evaluation setup");

    // Get transform between MoveIt and OctoMap frames
    geometry_msgs::msg::TransformStamped moveit_to_octomap_transform;
    try
    {
        moveit_to_octomap_transform = getMoveitToOctomapTransform(
            moveit_interface, octomap_interface, tf_buffer, node->get_logger());
    }
    catch (const tf2::TransformException&)
    {
        rclcpp::shutdown();
        return 1;
    }
    Eigen::Isometry3d transform_eigen = tf2::transformToEigen(moveit_to_octomap_transform.transform);
    // Check if the transform is identity (we must have aligned frames for this demo)
    if (!geometry_utils::isIdentityTransform(transform_eigen))
    {
        RCLCPP_ERROR(node->get_logger(), "MoveIt and OctoMap frames are not aligned, they must be the same for this demo");
        rclcpp::shutdown();
        return 1;
    }

    // Main NBV Planning Loop
    for (int iter = 0; iter < config.max_iterations; ++iter) {
        RCLCPP_INFO(node->get_logger(), "\n********** NBV Planning Iteration %d **********", iter + 1);

        // Find the frontiers
        std::vector<octomap::point3d> frontiers = octomap_interface->findFrontiers(
            config.min_unknown_neighbors, octomap_interface->hasBoundingBox());
        if (frontiers.empty()) {
            RCLCPP_WARN(node->get_logger(), "No frontiers found - exploration may be complete");
            break;
        }

        // Convert the frontiers from the map to the MoveIt frame
        std::vector<Eigen::Vector3d> frontiers_eigen = octomapVectorToEigen(frontiers);

        // Pre-filter the frontiers based on manipulation workspace
        std::vector<Eigen::Vector3d> viewable_frontiers;
        for (const auto& tff : frontiers_eigen) {
            if (manip_workspace->getDistance(tff) < config.ideal_camera_distance) {
                viewable_frontiers.push_back(tff);
            }
        }
        RCLCPP_DEBUG(node->get_logger(), "%zu out of %zu frontiers are viewable from the workspace",
            viewable_frontiers.size(), frontiers_eigen.size());
        if (viewable_frontiers.empty()) {
            RCLCPP_WARN(node->get_logger(), "No viewable frontiers found within manipulation workspace");
            break;
        }

        // Cluster frontiers
        int n_clusters = std::max(1, (int)viewable_frontiers.size() / 100);
        std::vector<Cluster> frontier_clusters = octomap_interface->kmeansCluster(
            eigenVectorToOctomap(viewable_frontiers), n_clusters, 50, 1e-4); // Convert eigen back to octomap points for clustering
        RCLCPP_DEBUG(node->get_logger(), "Clustered %zu viewable frontiers into %zu clusters",
            viewable_frontiers.size(), frontier_clusters.size());
        if (frontier_clusters.empty()) {
            RCLCPP_WARN(node->get_logger(), "No frontier clusters found after clustering");
            break;
        }
        if (visualizer)
            visualizer->publishClusteredVoxels(frontier_clusters, octomap_interface->getResolution(), 
                false, 0.8f, "frontier_clusters", moveit_interface->getPoseReferenceFrame());

        // Generate viewpoints
        // Generate spherical planar viewpoints
        std::vector<Viewpoint> spherical_planar_viewpoints = generatePlanarSphericalCapCandidates(
            init_cam_position, init_cam_orientation, 
            config.plane_half_extent, config.plane_spatial_resolution, 
            config.cap_max_theta_rad, config.cap_min_theta_rad);
        RCLCPP_DEBUG(node->get_logger(), "Generated %zu viewpoints from plane of spherical caps", spherical_planar_viewpoints.size());
        // Generate frontier-based viewpoints
        std::vector<Eigen::Vector3d> cluster_centers;
        for (const auto& cluster : frontier_clusters) {
            cluster_centers.push_back(octomapToEigen(cluster.center));
        }
        double min_distance = std::max(0.0, config.ideal_camera_distance - config.ideal_distance_tolerance);
        double max_distance = config.ideal_camera_distance + config.ideal_distance_tolerance;
        std::vector<Viewpoint> frontier_viewpoints = generateFrontierBasedViewpoints(
            cluster_centers, init_cam_orientation, min_distance, max_distance,
            config.num_viewpoints_per_frontier, false, config.z_bias_sigma, 0.05, 1000, node->get_logger());
        RCLCPP_DEBUG(node->get_logger(), "Generated %zu viewpoints from frontier clusters", frontier_viewpoints.size());
        // Combine viewpoints
        std::vector<Viewpoint> total_viewpoints;
        total_viewpoints.insert(total_viewpoints.end(), spherical_planar_viewpoints.begin(), spherical_planar_viewpoints.end());
        total_viewpoints.insert(total_viewpoints.end(), frontier_viewpoints.begin(), frontier_viewpoints.end());
        RCLCPP_INFO(node->get_logger(), "Total generated viewpoints: %zu", total_viewpoints.size());
        if (total_viewpoints.empty()) {
            RCLCPP_WARN(node->get_logger(), "No viewpoints generated!");
            break;
        }

        // Filter reachable viewpoints
        std::vector<Viewpoint> reachable_viewpoints = filterReachableViewpoints(total_viewpoints, manip_workspace, moveit_interface, config, node->get_logger());
        if (reachable_viewpoints.empty()) {
            RCLCPP_WARN(node->get_logger(), "No reachable viewpoints found!");
            break;
        }
        if (visualizer) {
            std::vector<geometry_msgs::msg::Pose> viewpoint_poses;
            for (const auto& vp : reachable_viewpoints) {
                viewpoint_poses.push_back(eigenToPose(vp.position, vp.orientation));
            }
            visualizer->publishCoordinates(viewpoint_poses, 0.1, 0.005, 0.5f, "reachable_viewpoints", moveit_interface->getPoseReferenceFrame());
        }

        // Compute utilities
        Eigen::Vector3d current_cam_position;
        std::array<double, 4> current_cam_orientation;
        moveit_interface->getLinkPose(config.camera_optical_link, current_cam_position, current_cam_orientation);
        double average_information_gain = 0.0;
        for (auto& vp : reachable_viewpoints) {
            vp.information_gain = computeInformationGain(vp, octomap_interface,
                geometry_utils::rad2Deg(config.camera_horizontal_fov_rad), geometry_utils::rad2Deg(config.camera_vertical_fov_rad),
                config.camera_width, config.camera_height, config.camera_max_range,
                octomap_interface->getResolution(), config.num_camera_rays, octomap_interface->hasBoundingBox(), node->get_logger(), nullptr);
            double distance = (vp.position - current_cam_position).norm();
            vp.cost = distance;
            vp.utility = vp.information_gain - config.alpha_cost_weight * distance;
            average_information_gain += vp.information_gain;
        }
        average_information_gain /= static_cast<double>(reachable_viewpoints.size());
        RCLCPP_DEBUG(node->get_logger(), "Average information gain across viewpoints: %.4f", average_information_gain);
    
        // Create max-heap
        std::priority_queue<Viewpoint> viewpoint_heap;
        for (const auto& vp : reachable_viewpoints)
            viewpoint_heap.push(vp);
    
        // Try viewpoints in order of utility
        Viewpoint best_viewpoint;
        std::optional<moveit::planning_interface::MoveGroupInterface::Plan> best_plan;
        while (!viewpoint_heap.empty()) {
            best_viewpoint = viewpoint_heap.top();
            viewpoint_heap.pop();
            // Check information gain threshold
            if (best_viewpoint.information_gain < config.min_information_gain) {
                RCLCPP_WARN(node->get_logger(), "No viewpoints left with information gain above threshold %.4f",
                        config.min_information_gain);
                break;
            }
            RCLCPP_DEBUG(node->get_logger(), "Trying viewpoint with utility %.4f, IG %.4f, cost %.4f",
                    best_viewpoint.utility, best_viewpoint.information_gain, best_viewpoint.cost);
            // Select best viewpoint with valid plan
            best_plan = planPathsToViewpoint(best_viewpoint, moveit_interface, config, node->get_logger());
            if (!best_plan) {
                RCLCPP_INFO(node->get_logger(), "Failed to plan to the best viewpoint, trying next best...");
                continue;
            }
            break;
        }

        if (!best_plan) {
            RCLCPP_WARN(node->get_logger(), "No valid viewpoint with path to IK solution found!");
            break;
        }

        // Execute motion if valid plan found
        if (!executeAndWaitForMotion(moveit_interface, node, *best_plan, node->get_logger())) {
            RCLCPP_ERROR(node->get_logger(), "Motion execution failed, ending NBV planning");
            break;
        }

        
        // Update the octomap after motion
        waitForOctomap(node, octomap_interface, trigger_clients, config, node->get_logger());

        // Evaluate if enabled
        if (config.enable_evaluation && octomap_interface->isSemanticTree()) {
            EvaluationMetrics eval_metrics;
            eval_metrics.time = node->now().seconds() - initial_time;  // Relative time since start
            auto latest_clusters = octomap_interface->clusterSemanticVoxels(false);
            auto match_result = octomap_interface->matchClustersToGroundTruth(latest_clusters, config.eval_threshold_radius, false);
            eval_metrics.class_metrics = octomap_interface->evaluateMatchResults(match_result, false);
            auto [occupied, free] = octomap_interface->getVoxelCounts();
            eval_metrics.occupied_voxels = occupied;
            eval_metrics.free_voxels = free;
            eval_metrics.bbox_coverage = octomap_interface->calculateCoverage();
            all_metrics.push_back(eval_metrics);
            // Print results
            RCLCPP_INFO(node->get_logger(), "\n=== Evaluation Results ===");
            RCLCPP_INFO(node->get_logger(), "Class ID | TP Clusters | FP Clusters | TP Points | FN Points");
            RCLCPP_INFO(node->get_logger(), "------------------------------------------------------------");
            for (const auto &cm : eval_metrics.class_metrics)
                RCLCPP_INFO(node->get_logger(), "  %6d | %11d | %12d | %9d | %9d", cm.class_id, cm.tp_clusters, cm.fp_clusters, cm.tp_points, cm.fn_points);
            RCLCPP_INFO(node->get_logger(), "------------------------------------------------------------");
            RCLCPP_INFO(node->get_logger(), "Class ID | Precision | Recall | F1 Score");
            RCLCPP_INFO(node->get_logger(), "------------------------------------------------------------");
            for (const auto &cm : eval_metrics.class_metrics)
                RCLCPP_INFO(node->get_logger(), "  %6d | %9.2f%% | %6.2f%% | %8.2f%%", cm.class_id, cm.precision * 100.0, cm.recall * 100.0, cm.f1_score * 100.0);
            // Visualize and log metrics
            if (visualizer) {
                visualizer->publishMatchResults(match_result, config.eval_threshold_radius * 2, 0.8f);
            }
            visualizer->plotAllMetrics(all_metrics, config.metrics_plots_dir);
            visualizer->logAllMetricsToCSV(all_metrics, config.metrics_data_dir);
        }

        // Clear visualization
        if (visualizer) {
            visualizer->clearAllMarkers();
        }

        // Check if there was an error in the loop
        if (!rclcpp::ok())
        {
            RCLCPP_INFO(node->get_logger(), "\nNBV planning interrupted by shutdown signal, exiting...");
            rclcpp::shutdown();
            return 1;
        }
    }

    // Return to initial joint configuration
    moveit_interface->clearPathConstraints();
    RCLCPP_INFO(node->get_logger(), "\nReturning to initial joint configuration...");
    if (!moveit_interface->planAndExecute(config.init_joint_angles_rad))
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to move to initial joint configuration");
        rclcpp::shutdown();
        return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Moved to initial joint configuration successfully");

    RCLCPP_INFO(node->get_logger(), "Press Ctrl+C to exit.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
