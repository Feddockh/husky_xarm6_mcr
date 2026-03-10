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
#include <filesystem>
#include <iomanip>
#include <sstream>

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

    // Service client to clear the occupancy map between runs
    auto clear_client = node->create_client<std_srvs::srv::Trigger>("/occupancy_map/clear");

    // Wait for sim time if needed
    if (node->get_parameter("use_sim_time").as_bool())
        waitForSimClock(node);

    // Load configuration (we will mutate metrics dirs per run if n_runs > 1)
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
    config.visualize = true;
    if (config.visualize)
        visualizer = std::make_shared<NBVVisualizer>(node, config.map_frame, config.visualization_topic);

    // Setup workspace with moveit interface (done once, reused)
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

    // Initialize octomap interface (subscriber stays alive across runs)
    RCLCPP_DEBUG(node->get_logger(), "\nInitializing OctoMap Interface");
    auto octomap_interface = std::make_shared<OctoMapInterface>(node, config.octomap_topic, true);

    // Handle camera triggering based on capture_type (done once)
    if (config.capture_type == "continuous")
    {
        startContinuousCapture(node, trigger_clients, node->get_logger());
    }
    else if (config.capture_type == "triggered")
    {
        if (!trigger_clients.send_trigger->wait_for_service(std::chrono::seconds(5)))
            RCLCPP_WARN(node->get_logger(), "send_trigger service not available, waiting for octomap anyway...");
    }

    // Clear occupancy map before starting run (includes run 1)
    if (!callClearMap(node, clear_client, node->get_logger()))
    {
        rclcpp::shutdown();
        return 1;
    }

    // Clear previous visualization markers
    if (visualizer)
    {
        visualizer->clearAllMarkers();
    }

    // Move to initial joint configuration
    RCLCPP_DEBUG(node->get_logger(), "\nMoving to initial joint configuration...");
    if (!moveit_interface->planToJointStateWithRetries(config.init_joint_angles_rad))
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to move to initial joint configuration");
        rclcpp::shutdown();
        return 1;
    }

    // Get initial camera pose wrt moveit base link (per run, in case robot moved)
    Eigen::Vector3d init_cam_position;
    std::array<double, 4> init_cam_orientation;
    if (!moveit_interface->getLinkPose(config.camera_optical_link, init_cam_position, init_cam_orientation))
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to get camera link pose");
        rclcpp::shutdown();
        return 1;
    }

    // Move to demo joint config for world 1
    // std::vector<double> demo_joint_angles_deg = {
    //     -5.0, 51.0, -67.0, -5.0, -74.0, 91.0
    // };
    // std::vector<double> demo_joint_angles_rad = geometry_utils::deg2Rad(demo_joint_angles_deg);
    // if (!moveit_interface->planToJointStateWithRetries(demo_joint_angles_rad))
    // {
    //     RCLCPP_ERROR(node->get_logger(), "Failed to move to demo joint configuration");
    //     rclcpp::shutdown();
    //     return 1;
    // }

    // Wait for initial octomap (fresh after clear)
    waitForOctomap(node, octomap_interface, trigger_clients, config, node->get_logger());

    // Main NBV Planning Loop
    RCLCPP_INFO(node->get_logger(), "\n********** NBV Paper Demo **********");

    // // Find the frontiers (volumetric)
    // std::vector<octomap::point3d> frontiers = octomap_interface->findFrontiers(
    //     config.min_unknown_neighbors, octomap_interface->hasBoundingBox());

    // // // Find the frontiers (semantic)
    // // std::vector<octomap::point3d> frontiers = octomap_interface->getUncertainVoxels(
    // //             config.max_semantic_certainty, true, true);

    // // Convert the frontiers from the map to the MoveIt frame
    // std::vector<Eigen::Vector3d> frontiers_eigen = octomapVectorToEigen(frontiers);

    // // Pre-filter the frontiers based on manipulation workspace
    // std::vector<Eigen::Vector3d> viewable_frontiers;
    // for (const auto& tff : frontiers_eigen) {
    //     if (manip_workspace->getDistance(tff) < config.ideal_camera_distance/2) {
    //         viewable_frontiers.push_back(tff);
    //     }
    // }

    // // Clustering frontiers
    // int n_clusters = std::max(1, (int)viewable_frontiers.size() / 40);
    // std::vector<Cluster> frontier_clusters = octomap_interface->kmeansCluster(
    //     eigenVectorToOctomap(viewable_frontiers), n_clusters, 50, 1e-4); // Convert eigen back to octomap points for clustering
    // RCLCPP_DEBUG(node->get_logger(), "Clustered %zu viewable frontiers into %zu clusters",
    //     viewable_frontiers.size(), frontier_clusters.size());
    // // if (visualizer)
    // //     visualizer->publishClusteredVoxels(frontier_clusters, octomap_interface->getResolution(), 
    // //         false, 0.8f, "frontier_clusters", moveit_interface->getPoseReferenceFrame());

    // Generate viewpoints
    auto [plane_corners_map, distance] = computePlane(octomap_interface, init_cam_position);
    if (visualizer)
    {
        RCLCPP_DEBUG(node->get_logger(), "Publishing NBV midplane for visualization");
        std_msgs::msg::ColorRGBA plane_color;
        plane_color.r = 0.0f; plane_color.g = 1.0f; plane_color.b = 0.0f; plane_color.a = 0.5f;
        visualizer->publishPlane(plane_corners_map, "nbv_plane", 0.02, plane_color);
    }

    // Generate viewpoints on the plane
    Eigen::Quaterniond init_cam_quat_map = geometry_utils::arrayToEigenQuat(init_cam_orientation);
    auto [plane_viewpoints, coverage_planes_map] = generateViewpointsFromPlane(
        plane_corners_map, distance, init_cam_quat_map, config.viewpoint_overlap_ratio, config.camera_horizontal_fov_rad, config.camera_vertical_fov_rad);
    
    if (visualizer)
    {
        RCLCPP_DEBUG(node->get_logger(), "Publishing coverage planes for visualization");
        for (size_t i = 0; i < coverage_planes_map.size(); ++i)
        {
            std_msgs::msg::ColorRGBA coverage_color;
            coverage_color.r = 0.0f;
            coverage_color.g = 0.5f;
            coverage_color.b = 1.0f;
            coverage_color.a = 0.3f;
            visualizer->publishPlane(coverage_planes_map[i], "coverage_" + std::to_string(i), 0.01, coverage_color);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }
    
    if (visualizer)
    {
        std::vector<Viewpoint> reachable_viewpoints = filterReachableViewpoints(plane_viewpoints, manip_workspace, moveit_interface, config, node->get_logger());
        std::vector<geometry_msgs::msg::Pose> viewpoint_poses;
        viewpoint_poses.reserve(reachable_viewpoints.size());
        for (const auto &vp : reachable_viewpoints)
        {
            viewpoint_poses.push_back(eigenToPose(vp.position, vp.orientation));
        }
        visualizer->publishCoordinates(
            viewpoint_poses, 0.15, 0.01, 0.5f, "plane_viewpoints", moveit_interface->getPoseReferenceFrame());
    }
    
    // Generate a spherical cap of viewpoints
    auto spherical_cap_viewpoints = generateSphericalCaps(plane_viewpoints, init_cam_orientation,
        config.cap_max_theta_rad, config.cap_min_theta_rad);

    if (visualizer)
    {
        std::vector<Viewpoint> reachable_viewpoints = filterReachableViewpoints(spherical_cap_viewpoints, manip_workspace, moveit_interface, config, node->get_logger());
        std::vector<geometry_msgs::msg::Pose> viewpoint_poses;
        viewpoint_poses.reserve(reachable_viewpoints.size());
        for (const auto &vp : reachable_viewpoints)
        {
            viewpoint_poses.push_back(eigenToPose(vp.position, vp.orientation));
        }
        visualizer->publishCoordinates(
            viewpoint_poses, 0.15, 0.01, 0.5f, "spherical_cap_viewpoints", moveit_interface->getPoseReferenceFrame());
    }

    // // Generate frontier-based viewpoints
    // std::vector<Eigen::Vector3d> cluster_centers;
    // for (const auto& cluster : frontier_clusters) {
    //     cluster_centers.push_back(octomapToEigen(cluster.center));
    // }

    // double min_distance = std::max(0.0, config.ideal_camera_distance - config.ideal_distance_tolerance);
    // double max_distance = config.ideal_camera_distance + config.ideal_distance_tolerance;
    // std::vector<Viewpoint> frontier_viewpoints = generateFrontierBasedViewpoints(
    //     cluster_centers, init_cam_orientation, min_distance, max_distance,
    //     config.num_viewpoints_per_frontier, false, config.z_bias_sigma, 0.05, 1000, node->get_logger());
    // RCLCPP_DEBUG(node->get_logger(), "Generated %zu viewpoints from frontier clusters", frontier_viewpoints.size());
    // if (visualizer)
    // {
    //     std::vector<geometry_msgs::msg::Pose> viewpoint_poses;
    //     viewpoint_poses.reserve(frontier_viewpoints.size());
    //     for (const auto &vp : frontier_viewpoints)
    //     {
    //         viewpoint_poses.push_back(eigenToPose(vp.position, vp.orientation));
    //     }
    //     visualizer->publishCoordinates(
    //         viewpoint_poses, 0.15, 0.01, 0.5f, "frontier_viewpoints", moveit_interface->getPoseReferenceFrame());
    // }

    // // Combine viewpoints
    // std::vector<Viewpoint> total_viewpoints;
    // total_viewpoints.insert(total_viewpoints.end(), spherical_cap_viewpoints.begin(), spherical_cap_viewpoints.end());
    // total_viewpoints.insert(total_viewpoints.end(), frontier_viewpoints.begin(), frontier_viewpoints.end());
    // RCLCPP_INFO(node->get_logger(), "Total generated viewpoints: %zu", total_viewpoints.size());

    // // Filter reachable viewpoints
    // std::vector<Viewpoint> reachable_viewpoints = filterReachableViewpoints(total_viewpoints, manip_workspace, moveit_interface, config, node->get_logger());
    // if (visualizer)
    // {
    //     std::vector<geometry_msgs::msg::Pose> viewpoint_poses;
    //     viewpoint_poses.reserve(reachable_viewpoints.size());
    //     for (const auto &vp : reachable_viewpoints)
    //     {
    //         viewpoint_poses.push_back(eigenToPose(vp.position, vp.orientation));
    //     }
    //     visualizer->publishCoordinates(
    //         viewpoint_poses, 0.15, 0.01, 0.5f, "reachable_viewpoints", moveit_interface->getPoseReferenceFrame());
    // }








    RCLCPP_INFO(node->get_logger(), "Press Ctrl+C to exit.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}