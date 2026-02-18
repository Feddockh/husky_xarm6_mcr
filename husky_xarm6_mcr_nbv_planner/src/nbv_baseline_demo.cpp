/**
 * @file nbv_baseline_demo.cpp
 * @brief Next-Best-View baseline demo for manipulation workspace
 */

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
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

using namespace husky_xarm6_mcr_nbv_planner;
using namespace husky_xarm6_mcr_nbv_planner::conversions;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "moveit_interface_node",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Wait for sim time if needed
    if (node->get_parameter("use_sim_time").as_bool())
        waitForSimClock(node);

    // Load configuration
    auto config = loadConfiguration(node);
    printConfiguration(config, node->get_logger());

    // Initialize MoveIt interface
    auto moveit_interface = setupMoveItInterface(node, config);
    RCLCPP_DEBUG(node->get_logger(), "MoveIt Interface initialized for group: %s with base link: %s",
                config.manipulator_group_name.c_str(), moveit_interface->getPoseReferenceFrame().c_str());

    // Initialize visualizer if requested
    std::shared_ptr<NBVVisualizer> visualizer;
    if (config.visualize)
        visualizer = std::make_shared<NBVVisualizer>(node, config.map_frame, config.visualization_topic);

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
        moveit_interface->getEndEffectorLink(),
        arrayToQuaternion(init_cam_orientation),
        M_PI_2, M_PI_2, M_PI_2);

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

    // Create trigger clients
    auto trigger_clients = createTriggerClients(node);

    // Handle camera triggering based on capture_type
    if (config.capture_type == "continuous")
    {
        startContinuousCapture(node, trigger_clients, node->get_logger());
    }
    else if (config.capture_type == "triggered")
    {
        stopVideoCapture(node, trigger_clients, node->get_logger());
        if (!trigger_clients.send_trigger->wait_for_service(std::chrono::seconds(5)))
            RCLCPP_WARN(node->get_logger(), "send_trigger service not available, waiting for octomap anyway...");
    }

    // Wait for initial octomap
    waitForOctomap(node, octomap_interface, trigger_clients, config, node->get_logger());

    // Initialize evaluation if enabled
    std::string csv_path = config.metrics_data_dir + "/nbv_metrics.csv";
    std::string plot_path = config.metrics_plots_dir + "/nbv_metrics_final.png";
    std::vector<std::vector<ClassMetrics>> all_metrics;
    if (config.enable_evaluation && octomap_interface->isSemanticTree())
    {
        if (!octomap_interface->loadGroundTruthSemantics(config.gt_points_file))
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to load ground truth file: %s", config.gt_points_file.c_str());
            rclcpp::shutdown();
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "Ground truth loaded successfully");
        // Perform initial evaluation
        auto latest_clusters = octomap_interface->clusterSemanticVoxels(false);
        auto match_result = octomap_interface->matchClustersToGroundTruth(latest_clusters, config.eval_threshold_radius, false);
        all_metrics.push_back(octomap_interface->evaluateMatchResults(match_result, false));
        if (visualizer)
        {
            visualizer->publishMatchResults(match_result, config.eval_threshold_radius * 2, 0.8f);
            visualizer->plotMetrics(all_metrics, "NBV Metrics", {}, plot_path);
        }
        visualizer->logMetricsToCSV(all_metrics, csv_path);
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

    RCLCPP_INFO(node->get_logger(), "\n=== Generate NBV Viewpoints ===");
    Eigen::Vector3d cam_pos_map = tf2::transformToEigen(moveit_to_octomap_transform.transform) * init_cam_position;
    auto [plane_corners_map, distance] = computePlane(octomap_interface, cam_pos_map);
    if (config.visualize && visualizer)
    {
        RCLCPP_DEBUG(node->get_logger(), "Publishing NBV midplane for visualization");
        std_msgs::msg::ColorRGBA plane_color;
        plane_color.r = 0.0f; plane_color.g = 1.0f; plane_color.b = 0.0f; plane_color.a = 0.5f;
        visualizer->publishPlane(plane_corners_map, "nbv_plane", 0.02, plane_color);
    }
    // Generate viewpoints on the plane
    Eigen::Isometry3d transform_eigen = tf2::transformToEigen(moveit_to_octomap_transform.transform);
    Eigen::Quaterniond init_cam_quat_map = Eigen::Quaterniond(transform_eigen.rotation()) * geometry_utils::arrayToEigenQuat(init_cam_orientation);
    double overlap_ratio = 0.45; // 45% overlap
    auto [all_viewpoints_map, coverage_planes_map] = generateViewpointsFromPlane(
        plane_corners_map, distance, init_cam_quat_map, overlap_ratio, config);
    if (config.visualize && visualizer)
    {
        RCLCPP_DEBUG(node->get_logger(), "Publishing coverage planes for visualization");
        for (size_t i = 0; i < coverage_planes_map.size(); ++i)
        {
            std_msgs::msg::ColorRGBA coverage_color;
            coverage_color.r = 0.0f; coverage_color.g = 0.5f; coverage_color.b = 1.0f; coverage_color.a = 0.3f;
            visualizer->publishPlane(coverage_planes_map[i], "coverage_" + std::to_string(i), 0.01, coverage_color);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    // Convert the viewpoints to moveit frame
    Eigen::Isometry3d transform_eigen_inverse = transform_eigen.inverse();
    std::vector<Viewpoint> all_viewpoints;
    for (const auto &vp_map : all_viewpoints_map)
    {
        Eigen::Vector3d pos_moveit = transform_eigen_inverse * vp_map.position;
        Eigen::Quaterniond ori_moveit = Eigen::Quaterniond(transform_eigen_inverse.rotation()) * geometry_utils::arrayToEigenQuat(vp_map.orientation);
        Viewpoint vp_moveit;
        vp_moveit.position = pos_moveit;
        vp_moveit.orientation = {ori_moveit.x(), ori_moveit.y(), ori_moveit.z(), ori_moveit.w()};
        all_viewpoints.push_back(vp_moveit);
    }

    RCLCPP_INFO(node->get_logger(), "\n=== Filter Reachable Viewpoints ===");
    auto reachable_viewpoints = filterReachableViewpoints(all_viewpoints, manip_workspace, moveit_interface, config, node->get_logger());
    if (reachable_viewpoints.empty()) {
        RCLCPP_ERROR(node->get_logger(), "No reachable viewpoints found");
        rclcpp::shutdown();
        return 1;
    }

    // Sort reachable viewpoints based on axis priority (relative to moveit frame)
    std::vector<std::string> axis_priority = {"-z", "-x"}; // Reading left to right and top to bottom
    sortViewpointsByAxisPriority(reachable_viewpoints, axis_priority);
    if (config.visualize && visualizer)
    {
        RCLCPP_INFO(node->get_logger(), "Publishing reachable viewpoints for visualization");
        for (size_t i = 0; i < reachable_viewpoints.size(); ++i)
        {
            visualizer->publishViewpoint(
                reachable_viewpoints[i], 0.2, 0.02, 1.0f,
                "vpoint_" + std::to_string(i), moveit_interface->getPoseReferenceFrame()); // in moveit frame
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    // Main NBV Planning Loop - use the filtered reachable_viewpoints from above
    for (size_t i = 0; i < reachable_viewpoints.size(); i++) {
        RCLCPP_INFO(node->get_logger(), "\n********** NBV Baseline Iteration %ld **********", i);

        // Select best viewpoint with valid plan
        auto plan = planPathToViewpoint(reachable_viewpoints[i], moveit_interface, config, node->get_logger());
        if (!plan) {
            RCLCPP_WARN(node->get_logger(), "Failed to plan to the best viewpoint, skipping iteration...");
            continue;
        }

        // Execute motion
        if (!executeAndWaitForMotion(moveit_interface, node, *plan, node->get_logger())) {
            RCLCPP_ERROR(node->get_logger(), "Motion execution failed, ending NBV planning");
            break;
        }

        // Wait for octomap update
        waitForOctomap(node, octomap_interface, trigger_clients, config, node->get_logger());

        // Evaluate if enabled
        if (config.enable_evaluation && octomap_interface->isSemanticTree()) {
            auto latest_clusters = octomap_interface->clusterSemanticVoxels(false);
            auto match_result = octomap_interface->matchClustersToGroundTruth(latest_clusters, config.eval_threshold_radius, false);
            auto metrics = octomap_interface->evaluateMatchResults(match_result, false);
            all_metrics.push_back(metrics);
            // Print results
            RCLCPP_INFO(node->get_logger(), "\n=== Evaluation Results ===");
            RCLCPP_INFO(node->get_logger(), "Class ID | TP Clusters | FP Clusters | TP Points | FN Points");
            RCLCPP_INFO(node->get_logger(), "------------------------------------------------------------");
            for (const auto &cm : metrics)
                RCLCPP_INFO(node->get_logger(), "  %6d | %11d | %12d | %9d | %9d", cm.class_id, cm.tp_clusters, cm.fp_clusters, cm.tp_points, cm.fn_points);
            RCLCPP_INFO(node->get_logger(), "------------------------------------------------------------");
            RCLCPP_INFO(node->get_logger(), "Class ID | Precision | Recall | F1 Score");
            RCLCPP_INFO(node->get_logger(), "------------------------------------------------------------");
            for (const auto &cm : metrics)
                RCLCPP_INFO(node->get_logger(), "  %6d | %9.2f%% | %6.2f%% | %8.2f%%", cm.class_id, cm.precision * 100.0, cm.recall * 100.0, cm.f1_score * 100.0);
            // Visualize and log metrics
            if (visualizer) {
                visualizer->publishMatchResults(match_result, config.eval_threshold_radius * 2, 0.8f);
                visualizer->plotMetrics(all_metrics, "NBV Metrics", {}, plot_path);
            }
            visualizer->logMetricsToCSV(all_metrics, csv_path);
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
