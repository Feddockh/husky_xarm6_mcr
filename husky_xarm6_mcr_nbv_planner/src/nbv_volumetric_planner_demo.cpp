/**
 * @file nbv_volumetric_planner_demo.cpp
 * @brief Next-Best-View volumetric planner demo for manipulation workspace
 * 
 * This file implements a modular NBV planner organized into discrete functions:
 * 
 * Configuration & Setup:
 *   - loadConfiguration()         - Load ROS parameters into config struct
 *   - printConfiguration()        - Display configuration
 *   - setupMoveItInterface()      - Initialize MoveIt planning interface
 *   - moveToInitialPose()         - Move manipulator to starting position
 *   - getInitialCameraPose()      - Get initial camera pose
 *   - setupWorkspace()            - Learn or load manipulation workspace
 * 
 * Camera Triggering:
 *   - createTriggerClients()      - Create ROS service clients for camera
 *   - startContinuousCapture()    - Start continuous video mode
 *   - stopVideoCapture()          - Stop video capture
 *   - waitForOctomapWithTriggers()- Wait for octomap with camera triggers
 *   - waitForOctomapUpdate()      - Wait for octomap to update after motion
 * 
 * Evaluation:
 *   - initializeEvaluation()      - Load GT and perform initial evaluation
 *   - performEvaluation()         - Evaluate semantic clustering
 * 
 * NBV Planning Steps:
 *   - computeAndClusterFrontiers()      - Find and cluster frontier voxels
 *   - generateAndFilterViewpoints()     - Generate and filter candidate views
 *   - computeViewpointUtilities()       - Compute IG, cost, and utility
 *   - selectBestViewpointWithPlan()     - Select best viewpoint with valid IK/plan
 *   - executeAndWaitForMotion()         - Execute motion and wait for completion
 * 
 * Main:
 *   - main()                      - Clean main loop orchestrating all steps
 */

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <chrono>
#include <queue>
#include <algorithm>
#include <optional>
#include "husky_xarm6_mcr_nbv_planner/moveit_interface.hpp"
#include "husky_xarm6_mcr_nbv_planner/manipulation_workspace.hpp"
#include "husky_xarm6_mcr_nbv_planner/octomap_interface.hpp"
#include "husky_xarm6_mcr_nbv_planner/nbv_visualizer.hpp"
#include "husky_xarm6_mcr_nbv_planner/viewpoint_generation.hpp"
#include "husky_xarm6_mcr_nbv_planner/geometry_utils.hpp"
#include "husky_xarm6_mcr_nbv_planner/conversions.hpp"

using namespace husky_xarm6_mcr_nbv_planner;
using namespace husky_xarm6_mcr_nbv_planner::conversions;

// ============================================================================
// Configuration Structure
// ============================================================================
struct NBVPlannerConfig {
    std::string manipulator_group_name;
    bool learn_workspace;
    std::string capture_type;
    std::string workspace_file;
    int num_samples;
    bool visualize;
    std::string visualization_topic;
    std::string octomap_topic;
    int max_iterations;
    double min_information_gain;
    double alpha_cost_weight;
    int num_viewpoints_per_frontier;
    double camera_horizontal_fov;
    double camera_vertical_fov;
    int camera_width;
    int camera_height;
    double camera_max_range;
    double ideal_camera_distance;
    int num_camera_rays;
    std::string map_frame;
    std::string camera_optical_link;
    std::string gt_points_file;
    bool enable_evaluation;
    double eval_threshold_radius;
};

struct TriggerClients {
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_video;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_video;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr send_trigger;
};

// ============================================================================
// Helper Functions
// ============================================================================
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

NBVPlannerConfig loadConfiguration(const std::shared_ptr<rclcpp::Node>& node)
{
    NBVPlannerConfig config;
    config.manipulator_group_name = node->get_parameter("manipulator_group_name").as_string();
    config.learn_workspace = node->get_parameter("learn_workspace").as_bool();
    config.capture_type = node->get_parameter("capture_type").as_string();
    config.workspace_file = node->get_parameter("manipulation_workspace_file").as_string();
    config.num_samples = node->get_parameter("num_samples").as_int();
    config.visualize = node->get_parameter("visualize").as_bool();
    config.visualization_topic = node->get_parameter("visualization_topic").as_string();
    config.octomap_topic = node->get_parameter("octomap_topic").as_string();
    config.max_iterations = node->get_parameter("max_iterations").as_int();
    config.min_information_gain = node->get_parameter("min_information_gain").as_double();
    config.alpha_cost_weight = node->get_parameter("alpha_cost_weight").as_double();
    config.num_viewpoints_per_frontier = node->get_parameter("num_viewpoints_per_frontier").as_int();
    config.camera_horizontal_fov = node->get_parameter("camera_horizontal_fov").as_double();
    config.camera_vertical_fov = node->get_parameter("camera_vertical_fov").as_double();
    config.camera_width = node->get_parameter("camera_width").as_int();
    config.camera_height = node->get_parameter("camera_height").as_int();
    config.camera_max_range = node->get_parameter("camera_max_range").as_double();
    config.ideal_camera_distance = node->get_parameter("ideal_camera_distance").as_double();
    config.num_camera_rays = node->get_parameter("num_camera_rays").as_int();
    config.map_frame = node->get_parameter("map_frame").as_string();
    config.camera_optical_link = node->get_parameter("camera_optical_link").as_string();
    config.gt_points_file = node->get_parameter("gt_points_file").as_string();
    config.enable_evaluation = node->get_parameter("enable_evaluation").as_bool();
    config.eval_threshold_radius = node->get_parameter("eval_threshold_radius").as_double();
    return config;
}

void printConfiguration(const NBVPlannerConfig& config, const rclcpp::Logger& logger)
{
    RCLCPP_INFO(logger, "\n=== NBV Volumetric Planner Configuration ===");
    RCLCPP_INFO(logger, "Manipulator group: %s", config.manipulator_group_name.c_str());
    RCLCPP_INFO(logger, "Learn workspace: %s", config.learn_workspace ? "true" : "false");
    RCLCPP_INFO(logger, "Capture type: %s", config.capture_type.c_str());
    RCLCPP_INFO(logger, "Workspace samples: %d", config.num_samples);
    RCLCPP_INFO(logger, "Visualization: %s", config.visualize ? "enabled" : "disabled");
    RCLCPP_INFO(logger, "Octomap topic: %s", config.octomap_topic.c_str());
    RCLCPP_INFO(logger, "Max iterations: %d", config.max_iterations);
    RCLCPP_INFO(logger, "Min information gain: %.4f", config.min_information_gain);
    RCLCPP_INFO(logger, "Alpha cost weight: %.4f", config.alpha_cost_weight);
    RCLCPP_INFO(logger, "Viewpoints per frontier: %d", config.num_viewpoints_per_frontier);
    RCLCPP_INFO(logger, "Camera H-FOV: %.1f rad (%.1f deg)", config.camera_horizontal_fov, config.camera_horizontal_fov * 180.0 / M_PI);
    RCLCPP_INFO(logger, "Camera V-FOV: %.1f rad (%.1f deg)", config.camera_vertical_fov, config.camera_vertical_fov * 180.0 / M_PI);
    RCLCPP_INFO(logger, "Camera resolution: %dx%d", config.camera_width, config.camera_height);
    RCLCPP_INFO(logger, "Max sensor range: %.2f m", config.camera_max_range);
    RCLCPP_INFO(logger, "Ideal camera distance: %.2f m", config.ideal_camera_distance);
    RCLCPP_INFO(logger, "Number of IG rays: %d", config.num_camera_rays);
    RCLCPP_INFO(logger, "Map frame: %s", config.map_frame.c_str());
    RCLCPP_INFO(logger, "GT points file: %s", config.gt_points_file.c_str());
    RCLCPP_INFO(logger, "Enable evaluation: %s", config.enable_evaluation ? "true" : "false");
    RCLCPP_INFO(logger, "Evaluation threshold radius: %.2f m", config.eval_threshold_radius);
    RCLCPP_INFO(logger, "============================================\n");
}

std::shared_ptr<MoveItInterface> setupMoveItInterface(
    const std::shared_ptr<rclcpp::Node>& node,
    const std::string& manipulator_group_name)
{
    auto interface = std::make_shared<MoveItInterface>(node, manipulator_group_name);
    interface->setPlanningPipelineId("ompl");
    interface->setPlannerId("RRTConnect");
    interface->setPlanningTime(1.0);
    interface->setNumPlanningAttempts(5);
    interface->setMaxVelocityScalingFactor(0.5);
    interface->setMaxAccelerationScalingFactor(0.5);
    
    RCLCPP_INFO(node->get_logger(), "Pipeline: %s", interface->getPlanningPipelineId().c_str());
    RCLCPP_INFO(node->get_logger(), "Planner ID: %s", interface->getPlannerId().c_str());
    RCLCPP_INFO(node->get_logger(), "Planning time: %.2f seconds", interface->getPlanningTime());
    RCLCPP_INFO(node->get_logger(), "Number of planning attempts: %d", interface->getNumPlanningAttempts());
    RCLCPP_INFO(node->get_logger(), "Max velocity scaling factor: %.2f", interface->getMaxVelocityScalingFactor());
    RCLCPP_INFO(node->get_logger(), "Max acceleration scaling factor: %.2f", interface->getMaxAccelerationScalingFactor());
    return interface;
}

bool moveToInitialPose(
    const std::shared_ptr<MoveItInterface>& interface,
    const std::vector<double>& init_joint_angles_rad,
    const rclcpp::Logger& logger)
{
    RCLCPP_INFO(logger, "\nMoving to initial joint configuration...");
    if (!interface->planAndExecute(init_joint_angles_rad)) {
        RCLCPP_ERROR(logger, "Failed to move to initial joint configuration");
        return false;
    }
    RCLCPP_INFO(logger, "Moved to initial joint configuration successfully");
    return true;
}

std::pair<Eigen::Vector3d, std::array<double, 4>> getInitialCameraPose(
    const std::shared_ptr<MoveItInterface>& interface,
    const std::string& camera_optical_link,
    const rclcpp::Logger& logger)
{
    Eigen::Vector3d position;
    std::array<double, 4> orientation;
    
    if (!interface->getLinkPose(camera_optical_link, position, orientation)) {
        RCLCPP_ERROR(logger, "Failed to get camera link pose");
        throw std::runtime_error("Failed to get camera link pose");
    }
    
    return {position, orientation};
}

std::shared_ptr<ManipulationWorkspace> setupWorkspace(
    const std::shared_ptr<MoveItInterface>& interface,
    const std::shared_ptr<NBVVisualizer>& visualizer,
    const NBVPlannerConfig& config,
    const rclcpp::Logger& logger)
{
    RCLCPP_INFO(logger, "\n=== Step 1: Manipulation Workspace ===");
    RCLCPP_INFO(logger, "Initializing Manipulation Workspace...");
    
    auto workspace = std::make_shared<ManipulationWorkspace>(interface, visualizer);
    
    if (config.learn_workspace) {
        RCLCPP_INFO(logger, "Learning workspace with %d samples...", config.num_samples);
        if (!workspace->learnWorkspace(config.num_samples, config.visualize)) {
            RCLCPP_ERROR(logger, "Failed to learn workspace");
            return nullptr;
        }
        RCLCPP_INFO(logger, "Learned %zu voxels, saving to: %s",
                    workspace->getNumReachableVoxels(), config.workspace_file.c_str());
        workspace->saveWorkspaceToFile(config.workspace_file);
    } else {
        RCLCPP_INFO(logger, "Loading workspace from: %s", config.workspace_file.c_str());
        if (!workspace->loadWorkspaceFromFile(config.workspace_file)) {
            RCLCPP_ERROR(logger, "Failed to load. Set learn_workspace:=true");
            return nullptr;
        }
        RCLCPP_INFO(logger, "Loaded %zu voxels", workspace->getNumReachableVoxels());
    }
    
    return workspace;
}

TriggerClients createTriggerClients(const std::shared_ptr<rclcpp::Node>& node)
{
    return {
        node->create_client<std_srvs::srv::Trigger>("/trigger/start_video"),
        node->create_client<std_srvs::srv::Trigger>("/trigger/stop_video"),
        node->create_client<std_srvs::srv::Trigger>("/trigger/send_trigger")
    };
}

bool startContinuousCapture(
    const std::shared_ptr<rclcpp::Node>& node,
    const TriggerClients& clients,
    const rclcpp::Logger& logger)
{
    RCLCPP_INFO(logger, "Starting continuous video capture...");
    if (!clients.start_video->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_WARN(logger, "start_video service not available, continuing anyway...");
        return false;
    }
    
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = clients.start_video->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(5)) == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(logger, "Continuous video capture started: %s", response->message.c_str());
            return true;
        } else {
            RCLCPP_WARN(logger, "Failed to start video: %s", response->message.c_str());
        }
    }
    return false;
}

bool stopVideoCapture(
    const std::shared_ptr<rclcpp::Node>& node,
    const TriggerClients& clients,
    const rclcpp::Logger& logger)
{
    RCLCPP_INFO(logger, "Using triggered capture mode, ensuring video is stopped...");
    if (!clients.stop_video->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_WARN(logger, "stop_video service not available, continuing anyway...");
        return false;
    }
    
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = clients.stop_video->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(5)) == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();
        RCLCPP_INFO(logger, "Stop video service called: %s", response->message.c_str());
        return true;
    }
    return false;
}

bool waitForOctomapWithTriggers(
    const std::shared_ptr<rclcpp::Node>& node,
    const std::shared_ptr<OctoMapInterface>& octomap_interface,
    const TriggerClients& clients,
    const std::string& capture_type,
    const std::string& octomap_topic,
    double timeout_seconds,
    const rclcpp::Logger& logger)
{
    RCLCPP_INFO(logger, "Waiting for octomap to be published on %s...", octomap_topic.c_str());
    
    rclcpp::Time start_time = node->now();
    rclcpp::Rate spin_rate(10);
    
    if (capture_type == "triggered") {
        // Triggered mode: send triggers periodically until octomap is available
        while (rclcpp::ok() && !octomap_interface->isTreeAvailable() && 
               (node->now() - start_time).seconds() < timeout_seconds) {
            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
            auto future = clients.send_trigger->async_send_request(request);
            RCLCPP_INFO(logger, "Sending camera trigger...");
            
            // Wait 1 second for trigger response and octomap
            auto wait_start = node->now();
            while (rclcpp::ok() && (node->now() - wait_start).seconds() < 1.0) {
                rclcpp::spin_some(node);
                if (octomap_interface->isTreeAvailable())
                    break;
                spin_rate.sleep();
            }
        }
    } else {
        // Continuous mode: just wait for octomap
        while (rclcpp::ok() && !octomap_interface->isTreeAvailable() && 
               (node->now() - start_time).seconds() < timeout_seconds) {
            rclcpp::spin_some(node);
            spin_rate.sleep();
        }
    }
    
    if (!octomap_interface->isTreeAvailable()) {
        RCLCPP_ERROR(logger, "Octomap not available after %.0f seconds. Exiting.", timeout_seconds);
        return false;
    }
    RCLCPP_INFO(logger, "Octomap received successfully");
    return true;
}

bool initializeEvaluation(
    const std::shared_ptr<OctoMapInterface>& octomap_interface,
    const std::shared_ptr<NBVVisualizer>& visualizer,
    std::vector<std::vector<ClassMetrics>>& all_metrics,
    const NBVPlannerConfig& config,
    const rclcpp::Logger& logger)
{
    if (!config.enable_evaluation || !octomap_interface->isSemanticTree()) {
        return true;  // Not an error, just not enabled
    }
    
    // Load ground truth
    if (config.gt_points_file.empty()) {
        RCLCPP_WARN(logger, "Evaluation enabled but no ground truth file specified");
        return false;
    }
    
    if (!octomap_interface->loadGroundTruthSemantics(config.gt_points_file)) {
        RCLCPP_ERROR(logger, "Failed to load ground truth file: %s", config.gt_points_file.c_str());
        return false;
    }
    RCLCPP_INFO(logger, "Ground truth loaded successfully");
    
    // Initial evaluation
    auto latest_clusters = octomap_interface->clusterSemanticVoxels(false);
    auto match_result = octomap_interface->matchClustersToGroundTruth(
        latest_clusters, config.eval_threshold_radius, false);
    
    if (visualizer) {
        visualizer->publishMatchResults(match_result, config.eval_threshold_radius * 2, 0.8f);
    }
    
    std::vector<ClassMetrics> eval_results = octomap_interface->evaluateMatchResults(match_result, false);
    all_metrics.push_back(eval_results);
    
    if (visualizer) {
        visualizer->plotMetrics(all_metrics, "NBV Metrics", {}, "./nbv_metrics_plot.png");
    }
    
    return true;
}

std::vector<Cluster> computeAndClusterFrontiers(
    const std::shared_ptr<OctoMapInterface>& octomap_interface,
    const std::shared_ptr<ManipulationWorkspace>& workspace,
    const std::shared_ptr<NBVVisualizer>& visualizer,
    const NBVPlannerConfig& config,
    const rclcpp::Logger& logger)
{
    RCLCPP_INFO(logger, "\n=== Step 3: Compute Frontiers ===");
    
    int min_unknown_neighbors = 1;
    bool use_frontier_bbox = true;
    std::vector<octomap::point3d> frontiers = octomap_interface->findFrontiers(
        min_unknown_neighbors, use_frontier_bbox);
    RCLCPP_INFO(logger, "Found %zu frontiers", frontiers.size());
    
    if (frontiers.empty()) {
        RCLCPP_WARN(logger, "No frontiers found - exploration may be complete or bbox too restrictive");
        return {};
    }
    
    // Filter viewable frontiers
    std::vector<octomap::point3d> viewable_frontiers;
    for (const auto& frontier : frontiers) {
        Eigen::Vector3d frontier_eigen(frontier.x(), frontier.y(), frontier.z());
        if (workspace->getDistance(frontier_eigen) < config.ideal_camera_distance) {
            viewable_frontiers.push_back(frontier);
        }
    }
    RCLCPP_INFO(logger, "%zu out of %zu frontiers are viewable from the workspace",
                viewable_frontiers.size(), frontiers.size());
    
    // Cluster
    int n_clusters = std::max(1, (int)viewable_frontiers.size() / 100);
    auto frontier_clusters = octomap_interface->kmeansCluster(viewable_frontiers, n_clusters, 50, 1e-4);
    RCLCPP_INFO(logger, "Clustered frontiers into %zu groups", frontier_clusters.size());
    
    if (visualizer) {
        double octomap_resolution = octomap_interface->getResolution();
        visualizer->publishClusteredVoxels(frontier_clusters, octomap_resolution, false, 0.8f, "frontier_clusters");
        RCLCPP_INFO(logger, "Frontier clusters visualized");
    }
    
    return frontier_clusters;
}

std::vector<Viewpoint> generateAndFilterViewpoints(
    const std::vector<Cluster>& frontier_clusters,
    const std::shared_ptr<ManipulationWorkspace>& workspace,
    const std::shared_ptr<NBVVisualizer>& visualizer,
    const Eigen::Vector3d& init_cam_position,
    const std::array<double, 4>& init_cam_orientation,
    const NBVPlannerConfig& config,
    const rclcpp::Logger& logger)
{
    RCLCPP_INFO(logger, "\n=== Step 4-5: Generate & Filter Viewpoints ===");
    
    // Generate spherical planar viewpoints
    std::vector<Viewpoint> spherical_planar_viewpoints = generatePlanarSphericalCapCandidates(
        init_cam_position, init_cam_orientation, 0.4, 0.2, 
        geometry_utils::deg2Rad(50.0), geometry_utils::deg2Rad(25.0));
    RCLCPP_INFO(logger, "Generated %zu viewpoints on planar spherical cap", spherical_planar_viewpoints.size());
    
    // Generate frontier-based viewpoints
    std::vector<Viewpoint> frontier_viewpoints;
    double best_range = 0.5;
    double range_tolerance = 0.1;
    for (const auto& cluster : frontier_clusters) {
        Eigen::Vector3d cluster_center(cluster.center.x(), cluster.center.y(), cluster.center.z());
        std::vector<Viewpoint> cluster_viewpoints = sampleViewsFromHemisphere(
            cluster_center,
            init_cam_orientation,
            best_range,
            best_range + range_tolerance,
            config.num_viewpoints_per_frontier,
            false,
            M_PI / 3.0,
            0.05,
            1000);
        frontier_viewpoints.insert(frontier_viewpoints.end(), cluster_viewpoints.begin(), cluster_viewpoints.end());
    }
    RCLCPP_INFO(logger, "Generated %zu viewpoints around %zu frontier clusters",
                frontier_viewpoints.size(), frontier_clusters.size());
    
    // Combine all viewpoints
    std::vector<Viewpoint> total_viewpoints;
    total_viewpoints.insert(total_viewpoints.end(), spherical_planar_viewpoints.begin(), spherical_planar_viewpoints.end());
    total_viewpoints.insert(total_viewpoints.end(), frontier_viewpoints.begin(), frontier_viewpoints.end());
    
    // Filter by reachability
    std::vector<Viewpoint> reachable_viewpoints;
    for (const auto& vp : total_viewpoints) {
        if (workspace->isPoseReachable(eigenToPose(vp.position, vp.orientation))) {
            reachable_viewpoints.push_back(vp);
        }
    }
    RCLCPP_INFO(logger, "%zu out of %zu viewpoints are reachable",
                reachable_viewpoints.size(), total_viewpoints.size());
    
    if (visualizer && !reachable_viewpoints.empty()) {
        std::vector<geometry_msgs::msg::Pose> viewpoint_poses;
        for (const auto& vp : reachable_viewpoints) {
            viewpoint_poses.push_back(eigenToPose(vp.position, vp.orientation));
        }
        visualizer->publishCoordinates(viewpoint_poses, 0.1, 0.005, 0.5f, "reachable_viewpoints");
        RCLCPP_INFO(logger, "Visualization complete!");
    }
    
    return reachable_viewpoints;
}

void computeViewpointUtilities(
    std::vector<Viewpoint>& viewpoints,
    const std::shared_ptr<OctoMapInterface>& octomap_interface,
    const Eigen::Vector3d& current_cam_position,
    const NBVPlannerConfig& config,
    const rclcpp::Logger& logger)
{
    RCLCPP_INFO(logger, "\n=== Step 6-7: Compute Information Gain & Utility ===");
    
    double octomap_resolution = octomap_interface->getResolution();
    double average_information_gain = 0.0;
    
    for (auto& vp : viewpoints) {
        vp.information_gain = computeInformationGain(
            vp, octomap_interface,
            geometry_utils::rad2Deg(config.camera_horizontal_fov),
            config.camera_width, config.camera_height,
            config.camera_max_range,
            octomap_resolution,
            config.num_camera_rays, true, logger, nullptr);
        
        double distance = (vp.position - current_cam_position).norm();
        vp.cost = distance;
        vp.utility = vp.information_gain - config.alpha_cost_weight * distance;
        average_information_gain += vp.information_gain;
    }
    
    average_information_gain /= static_cast<double>(viewpoints.size());
    RCLCPP_INFO(logger, "Average Information Gain across reachable viewpoints: %.4f", average_information_gain);
}

std::optional<std::pair<Viewpoint, moveit::planning_interface::MoveGroupInterface::Plan>>
selectBestViewpointWithPlan(
    const std::vector<Viewpoint>& viewpoints,
    const std::shared_ptr<MoveItInterface>& interface,
    const std::shared_ptr<NBVVisualizer>& visualizer,
    const NBVPlannerConfig& config,
    const rclcpp::Logger& logger)
{
    RCLCPP_INFO(logger, "\n=== Step 8: Select Best Viewpoint ===");
    
    std::vector<double> current_joint_state;
    if (!interface->getCurrentJointAngles(current_joint_state)) {
        RCLCPP_ERROR(logger, "Failed to get current joint state for IK seed");
        return std::nullopt;
    }
    
    // Create max-heap
    std::priority_queue<Viewpoint> viewpoint_heap;
    for (const auto& vp : viewpoints)
        viewpoint_heap.push(vp);
    
    // Try viewpoints in order of utility
    while (!viewpoint_heap.empty()) {
        auto candidate = viewpoint_heap.top();
        viewpoint_heap.pop();
        
        if (candidate.information_gain < config.min_information_gain) {
            RCLCPP_WARN(logger, "No viewpoints left with information gain above threshold %.4f",
                       config.min_information_gain);
            break;
        }
        
        RCLCPP_INFO(logger, "Trying viewpoint with utility %.4f, IG %.4f, cost %.4f",
                   candidate.utility, candidate.information_gain, candidate.cost);
        
        // Convert camera pose to end-effector pose
        geometry_msgs::msg::Pose cam_pose = eigenToPose(candidate.position, candidate.orientation);
        geometry_msgs::msg::Pose target_ee_pose;
        if (!interface->cameraPoseToEEPose(cam_pose, config.camera_optical_link, target_ee_pose)) {
            RCLCPP_DEBUG(logger, "  Failed to convert camera pose to EE pose, trying next viewpoint");
            continue;
        }
        
        // Compute IK
        auto best_joint_angles = interface->computeIK(current_joint_state, target_ee_pose, 0.01, 5);
        if (best_joint_angles.empty()) {
            RCLCPP_INFO(logger, "  No IK solution found for this viewpoint, trying next one...");
            continue;
        }
        
        // Check collision-free
        if (!interface->isStateValid(best_joint_angles)) {
            RCLCPP_INFO(logger, "  IK solution is in collision, trying next viewpoint...");
            continue;
        }
        RCLCPP_INFO(logger, "Found valid viewpoint with collision-free IK solution!");
        
        // Plan path
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        interface->planToJointGoal(best_joint_angles, plan);
        if (plan.trajectory_.joint_trajectory.points.empty()) {
            RCLCPP_INFO(logger, "  No reachable path to IK solution, trying next viewpoint...");
            continue;
        }
        
        RCLCPP_INFO(logger, "Found valid viewpoint with reachable path!");
        RCLCPP_INFO(logger, "Best Valid Viewpoint:");
        RCLCPP_INFO(logger, "  Position: [%.3f, %.3f, %.3f]",
                   candidate.position.x(), candidate.position.y(), candidate.position.z());
        RCLCPP_INFO(logger, "  Information Gain: %.4f", candidate.information_gain);
        RCLCPP_INFO(logger, "  Cost: %.4f", candidate.cost);
        RCLCPP_INFO(logger, "  Utility: %.4f", candidate.utility);
        
        if (visualizer) {
            visualizer->publishCoordinate(eigenToPose(candidate.position, candidate.orientation),
                                         0.15, 0.01, 1.0f, "best_viewpoint");
            RCLCPP_INFO(logger, "Best viewpoint published for visualization");
        }
        
        return std::make_pair(candidate, plan);
    }
    
    return std::nullopt;
}

bool executeAndWaitForMotion(
    const std::shared_ptr<MoveItInterface>& interface,
    const std::shared_ptr<rclcpp::Node>& node,
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const rclcpp::Logger& logger)
{
    RCLCPP_INFO(logger, "\n=== Step 9: Move to the Best Viewpoint ===");
    RCLCPP_INFO(logger, "Executing motion to best viewpoint...");
    
    if (!interface->execute(plan)) {
        RCLCPP_ERROR(logger, "Failed to execute motion to best viewpoint!");
        return false;
    }
    
    // Extract target joint angles from plan
    std::vector<double> target_joint_angles = plan.trajectory_.joint_trajectory.points.back().positions;
    
    // Wait for motion to complete
    rclcpp::Rate spin_rate(10);
    RCLCPP_INFO(logger, "Waiting for manipulator to reach goal...");
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        
        std::vector<double> current_joints;
        if (interface->getCurrentJointAngles(current_joints)) {
            double joint_error = 0.0;
            for (size_t i = 0; i < current_joints.size() && i < target_joint_angles.size(); ++i) {
                joint_error += std::abs(current_joints[i] - target_joint_angles[i]);
            }
            if (joint_error < 0.01) {
                RCLCPP_INFO(logger, "Manipulator reached goal position");
                RCLCPP_INFO(logger, "Successfully moved to best viewpoint!");
                return true;
            }
        }
        spin_rate.sleep();
    }
    return false;
}

void waitForOctomapUpdate(
    const std::shared_ptr<rclcpp::Node>& node,
    const std::shared_ptr<OctoMapInterface>& octomap_interface,
    const TriggerClients& clients,
    const std::string& capture_type,
    const rclcpp::Logger& logger)
{
    bool octomap_updated = false;
    rclcpp::Time initial_octomap_time = octomap_interface->getLastUpdateTime();
    double initial_time_seconds = initial_octomap_time.seconds();
    RCLCPP_INFO(logger, "Initial octomap timestamp: %.3f seconds", initial_time_seconds);
    
    rclcpp::Rate spin_rate(10);
    
    if (capture_type == "triggered") {
        // Triggered mode: send triggers until octomap updates
        RCLCPP_INFO(logger, "Sending camera triggers until octomap updates...");
        int trigger_count = 0;
        while (rclcpp::ok() && !octomap_updated) {
            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
            auto future = clients.send_trigger->async_send_request(request);
            trigger_count++;
            RCLCPP_INFO(logger, "Sending camera trigger #%d...", trigger_count);
            
            // Wait ~1 second and check for octomap update
            auto wait_start = node->now();
            while (rclcpp::ok() && (node->now() - wait_start).seconds() < 1.0) {
                rclcpp::spin_some(node);
                rclcpp::Time new_octomap_time = octomap_interface->getLastUpdateTime();
                double new_time_seconds = new_octomap_time.seconds();
                
                if (new_time_seconds != initial_time_seconds) {
                    octomap_updated = true;
                    RCLCPP_INFO(logger, "Octomap has been updated! New timestamp: %.3f seconds (delta: %.3f)",
                               new_time_seconds, new_time_seconds - initial_time_seconds);
                    break;
                }
                spin_rate.sleep();
            }
            
            if (!octomap_updated) {
                RCLCPP_INFO(logger, "No octomap update detected yet, will send another trigger...");
            }
        }
    } else {
        // Continuous mode: just wait for octomap update
        while (rclcpp::ok() && !octomap_updated) {
            rclcpp::spin_some(node);
            rclcpp::Time new_octomap_time = octomap_interface->getLastUpdateTime();
            double new_time_seconds = new_octomap_time.seconds();
            
            if (new_time_seconds != initial_time_seconds) {
                octomap_updated = true;
                RCLCPP_INFO(logger, "Octomap has been updated with new sensor data");
            }
            spin_rate.sleep();
        }
    }
}

void performEvaluation(
    const std::shared_ptr<OctoMapInterface>& octomap_interface,
    const std::shared_ptr<NBVVisualizer>& visualizer,
    std::vector<std::vector<ClassMetrics>>& all_metrics,
    const NBVPlannerConfig& config,
    [[maybe_unused]] const rclcpp::Logger& logger)
{
    auto latest_clusters = octomap_interface->clusterSemanticVoxels(false);
    auto match_result = octomap_interface->matchClustersToGroundTruth(
        latest_clusters, config.eval_threshold_radius, false);
    
    if (visualizer) {
        visualizer->publishMatchResults(match_result, config.eval_threshold_radius * 2, 0.8f);
    }
    
    std::vector<ClassMetrics> eval_results = octomap_interface->evaluateMatchResults(match_result, false);
    all_metrics.push_back(eval_results);
    
    if (visualizer) {
        visualizer->plotMetrics(all_metrics, "NBV Metrics", {}, "./nbv_metrics_plot.png");
    }
}

// ============================================================================
// Main Function
// ============================================================================
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

    // Initialize MoveIt interface
    auto interface = setupMoveItInterface(node, config.manipulator_group_name);

    // Initialize visualizer if requested
    std::shared_ptr<NBVVisualizer> visualizer;
    if (config.visualize) {
        visualizer = std::make_shared<NBVVisualizer>(node, config.map_frame, config.visualization_topic);
        RCLCPP_INFO(node->get_logger(), "Visualizer initialized on topic: %s", config.visualization_topic.c_str());
    }

    // Move to initial pose
    std::vector<double> init_joint_angles_rad = geometry_utils::deg2Rad({0.0, -45.0, -45.0, 0.0, 0.0, 90.0});
    if (!moveToInitialPose(interface, init_joint_angles_rad, node->get_logger())) {
        rclcpp::shutdown();
        return 1;
    }

    // Get initial camera pose
    auto [init_cam_position, init_cam_orientation] = getInitialCameraPose(
        interface, config.camera_optical_link, node->get_logger());

    // Set orientation constraints (±90 degrees tolerance)
    interface->setOrientationConstraints(
        interface->getEndEffectorLink(),
        arrayToQuaternion(init_cam_orientation),
        M_PI_2, M_PI_2, M_PI_2);

    // Setup workspace
    auto workspace = setupWorkspace(interface, visualizer, config, node->get_logger());
    if (!workspace) {
        rclcpp::shutdown();
        return 1;
    }

    // Initialize octomap interface
    RCLCPP_INFO(node->get_logger(), "\n=== Step 2: Receive OctoMap ===");
    RCLCPP_INFO(node->get_logger(), "Initializing OctoMap Interface...");
    auto octomap_interface = std::make_shared<OctoMapInterface>(node, config.octomap_topic, true);

    // Create trigger clients
    auto trigger_clients = createTriggerClients(node);

    // Handle camera triggering based on capture_type
    if (config.capture_type == "continuous") {
        startContinuousCapture(node, trigger_clients, node->get_logger());
    } else if (config.capture_type == "triggered") {
        stopVideoCapture(node, trigger_clients, node->get_logger());
        if (!trigger_clients.send_trigger->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_WARN(node->get_logger(), "send_trigger service not available, waiting for octomap anyway...");
        }
    }

    // Wait for initial octomap
    if (!waitForOctomapWithTriggers(node, octomap_interface, trigger_clients, 
                                    config.capture_type, config.octomap_topic,
                                    30.0, node->get_logger())) {
        rclcpp::shutdown();
        return 1;
    }

    // Initialize evaluation if enabled
    std::vector<std::vector<ClassMetrics>> all_metrics;
    if (!initializeEvaluation(octomap_interface, visualizer, all_metrics, config, node->get_logger())) {
        RCLCPP_WARN(node->get_logger(), "Evaluation initialization failed, continuing without evaluation");
        config.enable_evaluation = false;
    }

    // Main NBV Planning Loop
    for (int iter = 0; iter < config.max_iterations; ++iter) {
        RCLCPP_INFO(node->get_logger(), "\n********** NBV Planning Iteration %d **********", iter + 1);

        // Compute and cluster frontiers
        auto frontier_clusters = computeAndClusterFrontiers(
            octomap_interface, workspace, visualizer, config, node->get_logger());
        if (frontier_clusters.empty()) {
            break;
        }

        // Generate and filter viewpoints
        auto reachable_viewpoints = generateAndFilterViewpoints(
            frontier_clusters, workspace, visualizer, init_cam_position, init_cam_orientation,
            config, node->get_logger());
        if (reachable_viewpoints.empty()) {
            RCLCPP_WARN(node->get_logger(), "No reachable viewpoints generated!");
            break;
        }

        // Compute utilities
        Eigen::Vector3d current_cam_position;
        std::array<double, 4> current_cam_orientation;
        interface->getLinkPose(config.camera_optical_link, current_cam_position, current_cam_orientation);
        computeViewpointUtilities(reachable_viewpoints, octomap_interface, current_cam_position,
                                 config, node->get_logger());

        // Select best viewpoint with valid plan
        auto best_result = selectBestViewpointWithPlan(
            reachable_viewpoints, interface, visualizer, config, node->get_logger());
        if (!best_result) {
            RCLCPP_WARN(node->get_logger(), "No valid viewpoint with path to IK solution found!");
            break;
        }

        auto [best_viewpoint, plan] = *best_result;

        // Execute motion
        if (!executeAndWaitForMotion(interface, node, plan, node->get_logger())) {
            RCLCPP_ERROR(node->get_logger(), "Motion execution failed, ending NBV planning");
            break;
        }

        // Wait for octomap update
        waitForOctomapUpdate(node, octomap_interface, trigger_clients, 
                            config.capture_type, node->get_logger());

        // Evaluate if enabled
        if (config.enable_evaluation && octomap_interface->isSemanticTree()) {
            performEvaluation(octomap_interface, visualizer, all_metrics, config, node->get_logger());
        }

        // Clear visualization
        if (visualizer) {
            visualizer->clearAllMarkers();
            RCLCPP_INFO(node->get_logger(), "Cleared all visualization markers");
        }
    }

    // Return to initial pose
    RCLCPP_INFO(node->get_logger(), "\nMoving back to initial joint configuration...");
    if (!moveToInitialPose(interface, init_joint_angles_rad, node->get_logger())) {
        RCLCPP_WARN(node->get_logger(), "Failed to move back to initial joint configuration");
    } else {
        RCLCPP_INFO(node->get_logger(), "Successfully returned to initial joint configuration");
    }

    RCLCPP_INFO(node->get_logger(), "Press Ctrl+C to exit.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
