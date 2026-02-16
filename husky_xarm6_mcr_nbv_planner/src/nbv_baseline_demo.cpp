/**
 * @file nbv_baseline_demo.cpp
 * @brief Next-Best-View baseline demo for manipulation workspace
 * 
 * This file implements a modular NBV planner organized into discrete functions:
 * 
 * Configuration & Setup:
 *   - loadConfiguration()         - Load ROS parameters into config struct
 *   - printConfiguration()        - Display configuration
 *   - setupMoveItInterface()      - Initialize MoveIt planning interface
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
 *   - generateViewpoints()              - Generate candidate views
 *   - selectNextViewpointWithPlan()     - Select best viewpoint with valid IK/plan
 *   - executeAndWaitForMotion()         - Execute motion and wait for completion
 * 
 * Main:
 *   - main()                      - Clean main loop orchestrating all steps
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
    std::string metrics_plots_dir;
    std::string metrics_data_dir;
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
    config.camera_horizontal_fov = node->get_parameter("camera_horizontal_fov").as_double() * M_PI / 180.0;  // Convert degrees to radians
    config.camera_vertical_fov = node->get_parameter("camera_vertical_fov").as_double() * M_PI / 180.0;      // Convert degrees to radians
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
    config.metrics_plots_dir = node->get_parameter("metrics_plots_dir").as_string();
    config.metrics_data_dir = node->get_parameter("metrics_data_dir").as_string();
    return config;
}

void printConfiguration(const NBVPlannerConfig& config, const rclcpp::Logger& logger)
{
    RCLCPP_INFO(logger, "\n=== NBV Baseline Configuration ===");
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
    RCLCPP_INFO(logger, "Metrics plots directory: %s", config.metrics_plots_dir.c_str());
    RCLCPP_INFO(logger, "Metrics data directory: %s", config.metrics_data_dir.c_str());
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

std::shared_ptr<ManipulationWorkspace> setupWorkspace(
    const std::shared_ptr<MoveItInterface>& interface,
    const std::shared_ptr<NBVVisualizer>& visualizer,
    const NBVPlannerConfig& config,
    const rclcpp::Logger& logger)
{
    RCLCPP_INFO(logger, "\n=== Manipulation Workspace ===");
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
        std::string plot_path = config.metrics_plots_dir + "/nbv_metrics_initial.png";
        visualizer->plotMetrics(all_metrics, "NBV Metrics - Initial", {}, plot_path);
    }
    
    return true;
}

std::vector<Viewpoint> generateViewpoints(
    const std::shared_ptr<ManipulationWorkspace>& workspace,
    const std::shared_ptr<MoveItInterface>& moveit_interface,
    const std::shared_ptr<OctoMapInterface>& octomap_interface,
    const std::shared_ptr<NBVVisualizer>& visualizer,
    const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
    const Eigen::Vector3d& init_cam_position,
    const std::array<double, 4>& init_cam_orientation,
    const double overlap_ratio,
    const bool use_current_distance,
    const NBVPlannerConfig& config,
    const rclcpp::Logger& logger)
{
    RCLCPP_INFO(logger, "\n=== Generate Viewpoints ===");

    // if(visualizer) {
    //     std_msgs::msg::ColorRGBA color;
    //     color.r = 0.0f; color.g = 1.0f; color.b = 0.0f; color.a = 0.5f;
    //     visualizer->publishBoundingBox(eigenToOctomap(min_bbox), eigenToOctomap(max_bbox), "bbox", 0.02, color, frame_id);
    // }


    octomap::point3d min_bbox_octo, max_bbox_octo;
    octomap_interface->getBoundingBox(min_bbox_octo, max_bbox_octo); // In octomap map frame
    Eigen::Vector3d min_bbox = octomapToEigen(min_bbox_octo);
    Eigen::Vector3d max_bbox = octomapToEigen(max_bbox_octo);

    // Extract basis vectors from orientation (wrt to the moveit base frame)
    const Eigen::Quaterniond base_quat = geometry_utils::arrayToEigenQuat(init_cam_orientation);
    const Eigen::Matrix3d R_base = base_quat.toRotationMatrix();
    Eigen::Vector3d plane_x = R_base.col(0);
    Eigen::Vector3d plane_y = R_base.col(1);
    Eigen::Vector3d normal = R_base.col(2);

    // Convert basis vectors to octomap frame if needed
    if (moveit_interface->getPoseReferenceFrame() != octomap_interface->getOctomapFrameId()) {
        try {
            geometry_msgs::msg::TransformStamped transform_stamped = 
                tf_buffer->lookupTransform(
                    octomap_interface->getOctomapFrameId(),
                    moveit_interface->getPoseReferenceFrame(),
                    tf2::TimePointZero);
            
            Eigen::Isometry3d transform = tf2::transformToEigen(transform_stamped.transform);
            Eigen::Matrix3d R_map = transform.rotation();
            
            // Transform basis vectors
            plane_x = R_map * plane_x;
            plane_y = R_map * plane_y;
            normal = R_map * normal;
            
            RCLCPP_INFO(logger, "Transformed basis vectors from %s to %s",
                       moveit_interface->getPoseReferenceFrame().c_str(),
                       octomap_interface->getOctomapFrameId().c_str());
        } catch (const tf2::TransformException& ex) {
            RCLCPP_ERROR(logger, "TF2 exception: %s", ex.what());
            return {};
        }
    }

    // Define viewpoint generation parameters (wrt to the octomap frame)
    const Eigen::Vector3d mid_point = 0.5 * (min_bbox + max_bbox);

    // Find intersection of plane with bounding box edges
    // The plane passes through mid_point with normal vector 'normal'
    // Plane equation: normal · (P - mid_point) = 0
    auto intersectPlaneEdge = [&](const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) -> std::pair<bool, Eigen::Vector3d> {
        // Edge from p1 to p2: P(t) = p1 + t*(p2-p1), t in [0,1]
        // Plane: normal · (P - mid_point) = 0
        // Solve: normal · (p1 + t*(p2-p1) - mid_point) = 0
        Eigen::Vector3d edge_dir = p2 - p1;
        double denom = normal.dot(edge_dir);
        
        if (std::abs(denom) < 1e-9) {
            return {false, Eigen::Vector3d::Zero()};  // Edge parallel to plane
        }
        
        double t = normal.dot(mid_point - p1) / denom;
        
        if (t < -1e-9 || t > 1.0 + 1e-9) {
            return {false, Eigen::Vector3d::Zero()};  // Intersection outside edge
        }
        
        return {true, p1 + t * edge_dir};
    };
    
    // Define all 12 edges of the bounding box
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> bbox_edges = {
        // Bottom face (z = min)
        {Eigen::Vector3d(min_bbox.x(), min_bbox.y(), min_bbox.z()), Eigen::Vector3d(max_bbox.x(), min_bbox.y(), min_bbox.z())},
        {Eigen::Vector3d(max_bbox.x(), min_bbox.y(), min_bbox.z()), Eigen::Vector3d(max_bbox.x(), max_bbox.y(), min_bbox.z())},
        {Eigen::Vector3d(max_bbox.x(), max_bbox.y(), min_bbox.z()), Eigen::Vector3d(min_bbox.x(), max_bbox.y(), min_bbox.z())},
        {Eigen::Vector3d(min_bbox.x(), max_bbox.y(), min_bbox.z()), Eigen::Vector3d(min_bbox.x(), min_bbox.y(), min_bbox.z())},
        // Top face (z = max)
        {Eigen::Vector3d(min_bbox.x(), min_bbox.y(), max_bbox.z()), Eigen::Vector3d(max_bbox.x(), min_bbox.y(), max_bbox.z())},
        {Eigen::Vector3d(max_bbox.x(), min_bbox.y(), max_bbox.z()), Eigen::Vector3d(max_bbox.x(), max_bbox.y(), max_bbox.z())},
        {Eigen::Vector3d(max_bbox.x(), max_bbox.y(), max_bbox.z()), Eigen::Vector3d(min_bbox.x(), max_bbox.y(), max_bbox.z())},
        {Eigen::Vector3d(min_bbox.x(), max_bbox.y(), max_bbox.z()), Eigen::Vector3d(min_bbox.x(), min_bbox.y(), max_bbox.z())},
        // Vertical edges
        {Eigen::Vector3d(min_bbox.x(), min_bbox.y(), min_bbox.z()), Eigen::Vector3d(min_bbox.x(), min_bbox.y(), max_bbox.z())},
        {Eigen::Vector3d(max_bbox.x(), min_bbox.y(), min_bbox.z()), Eigen::Vector3d(max_bbox.x(), min_bbox.y(), max_bbox.z())},
        {Eigen::Vector3d(max_bbox.x(), max_bbox.y(), min_bbox.z()), Eigen::Vector3d(max_bbox.x(), max_bbox.y(), max_bbox.z())},
        {Eigen::Vector3d(min_bbox.x(), max_bbox.y(), min_bbox.z()), Eigen::Vector3d(min_bbox.x(), max_bbox.y(), max_bbox.z())}
    };
    
    // Find all intersection points
    std::vector<Eigen::Vector3d> intersection_points;
    for (const auto& [p1, p2] : bbox_edges) {
        auto [intersects, point] = intersectPlaneEdge(p1, p2);
        if (intersects) {
            intersection_points.push_back(point);
        }
    }
    
    if (intersection_points.size() < 3) {
        RCLCPP_ERROR(logger, "Plane does not intersect bounding box properly (found %zu points)", 
                     intersection_points.size());
        return {};
    }
    
    // Sort intersection points by angle around the plane center to form a convex polygon
    std::vector<Eigen::Vector3d> corners;
    std::vector<std::pair<double, Eigen::Vector3d>> angle_points;
    for (const auto& pt : intersection_points) {
        Eigen::Vector3d rel = pt - mid_point;
        double angle = std::atan2(rel.dot(plane_y), rel.dot(plane_x));
        angle_points.push_back({angle, pt});
    }
    std::sort(angle_points.begin(), angle_points.end(), 
              [](const auto& a, const auto& b) { return a.first < b.first; });
    for (const auto& [angle, pt] : angle_points) {
        corners.push_back(pt);
    }
    
    // Compute plane dimensions (bounding rectangle in plane coordinates)
    double plane_x_min = std::numeric_limits<double>::max();
    double plane_x_max = -std::numeric_limits<double>::max();
    double plane_y_min = std::numeric_limits<double>::max();
    double plane_y_max = -std::numeric_limits<double>::max();
    
    for (const auto& corner : corners) {
        Eigen::Vector3d rel = corner - mid_point;
        double x_proj = rel.dot(plane_x);
        double y_proj = rel.dot(plane_y);
        plane_x_min = std::min(plane_x_min, x_proj);
        plane_x_max = std::max(plane_x_max, x_proj);
        plane_y_min = std::min(plane_y_min, y_proj);
        plane_y_max = std::max(plane_y_max, y_proj);
    }
    
    double plane_width = plane_x_max - plane_x_min;
    double plane_height = plane_y_max - plane_y_min;

    RCLCPP_INFO(logger, "Plane through midpoint (clipped to bbox):");
    RCLCPP_INFO(logger, "  Midpoint: [%.3f, %.3f, %.3f]", mid_point.x(), mid_point.y(), mid_point.z());
    RCLCPP_INFO(logger, "  Number of corners: %zu", corners.size());
    RCLCPP_INFO(logger, "  Plane dimensions: %.3f m × %.3f m", plane_width, plane_height);
    for (size_t i = 0; i < corners.size(); ++i) {
        RCLCPP_INFO(logger, "  Corner %zu: [%.3f, %.3f, %.3f]", i+1, 
                    corners[i].x(), corners[i].y(), corners[i].z());
    }
    
    // Visualize the plane in RViz
    if (visualizer) {
        std::vector<octomap::point3d> viz_corners;
        for (size_t i = 0; i < corners.size(); ++i) {
            viz_corners.push_back(eigenToOctomap(corners[i]));
        }
        std_msgs::msg::ColorRGBA color;
        color.r = 1.0f; color.g = 0.0f; color.b = 0.0f; color.a = 0.5f;
        visualizer->publishPlane(viz_corners, "plane", 0.02, color);
        RCLCPP_INFO(logger, "Viewpoint generation plane published for visualization");
    }

    // Now that we have the plane, generate a grid of viewpoints on it each overlapping by the specified ratio
    const double hfov = config.camera_horizontal_fov;
    const double vfov = config.camera_vertical_fov;
    
    // Calculate distance to use based on parameter
    double distance;
    if (use_current_distance) {
        // Transform init_cam_position to map frame if needed
        Eigen::Vector3d cam_pos_map = init_cam_position;
        if (moveit_interface->getPoseReferenceFrame() != octomap_interface->getOctomapFrameId()) {
            try {
                geometry_msgs::msg::TransformStamped transform_stamped = 
                    tf_buffer->lookupTransform(
                        octomap_interface->getOctomapFrameId(),
                        moveit_interface->getPoseReferenceFrame(),
                        tf2::TimePointZero);
                Eigen::Isometry3d transform = tf2::transformToEigen(transform_stamped.transform);
                cam_pos_map = transform * init_cam_position;
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN(logger, "Failed to transform camera position, using ideal distance instead: %s", ex.what());
                distance = config.ideal_camera_distance;
            }
        }
        distance = (cam_pos_map - mid_point).norm();
        RCLCPP_INFO(logger, "Using current camera distance: %.3f m", distance);
    } else {
        distance = config.ideal_camera_distance;
        RCLCPP_INFO(logger, "Using ideal camera distance: %.3f m", distance);
    }
    
    // Calculate the size of the square footprint on the plane at the ideal distance
    double coverage_width = 2.0 * distance * std::abs(std::tan(hfov / 2.0));
    double coverage_height = 2.0 * distance * std::abs(std::tan(vfov / 2.0));
    
    RCLCPP_INFO(logger, "Camera FOV: H=%.3f rad (%.1f°), V=%.3f rad (%.1f°)", 
                hfov, hfov * 180.0 / M_PI, vfov, vfov * 180.0 / M_PI);
    RCLCPP_INFO(logger, "Camera coverage at %.3f m: %.3f m × %.3f m", 
                distance, coverage_width, coverage_height);
    
    // Calculate spacing with overlap
    double spacing_x = coverage_width * (1.0 - overlap_ratio);
    double spacing_y = coverage_height * (1.0 - overlap_ratio);
    
    RCLCPP_INFO(logger, "Grid spacing with %.0f%% overlap: %.3f m × %.3f m",
                overlap_ratio * 100.0, spacing_x, spacing_y);
    
    // Calculate number of grid points needed to cover the plane
    int num_x = std::max(1, static_cast<int>(std::ceil(plane_width / spacing_x)));
    int num_y = std::max(1, static_cast<int>(std::ceil(plane_height / spacing_y)));
    
    RCLCPP_INFO(logger, "Grid dimensions: %d × %d = %d viewpoints", num_x, num_y, num_x * num_y);
    
    // Generate grid points centered on the plane (in plane local coordinates)
    std::vector<Eigen::Vector3d> grid_points_on_plane;
    double start_x = -(num_x - 1) * spacing_x / 2.0;
    double start_y = -(num_y - 1) * spacing_y / 2.0;
    
    for (int i = 0; i < num_x; ++i) {
        for (int j = 0; j < num_y; ++j) {
            double local_x = start_x + i * spacing_x;
            double local_y = start_y + j * spacing_y;
            
            // Convert to 3D world position on the plane (in map frame)
            Eigen::Vector3d point_on_plane = mid_point + local_x * plane_x + local_y * plane_y;
            grid_points_on_plane.push_back(point_on_plane);
        }
    }

    // Visualize grid coverage areas on the plane and the viewpoints
    if (visualizer) {
        double half_coverage_width = coverage_width / 2.0;
        double half_coverage_height = coverage_height / 2.0;
        
        for (size_t i = 0; i < grid_points_on_plane.size(); ++i) {
            const Eigen::Vector3d& center = grid_points_on_plane[i];
            
            // Create 4 corners of the coverage rectangle
            std::vector<octomap::point3d> coverage_corners;
            coverage_corners.push_back(eigenToOctomap(center + half_coverage_width * plane_x + half_coverage_height * plane_y));
            coverage_corners.push_back(eigenToOctomap(center - half_coverage_width * plane_x + half_coverage_height * plane_y));
            coverage_corners.push_back(eigenToOctomap(center - half_coverage_width * plane_x - half_coverage_height * plane_y));
            coverage_corners.push_back(eigenToOctomap(center + half_coverage_width * plane_x - half_coverage_height * plane_y));
            
            std_msgs::msg::ColorRGBA color;
            color.r = 0.0f; color.g = 0.5f; color.b = 1.0f; color.a = 0.3f;
            std::string ns = "grid_coverage_" + std::to_string(i);
            visualizer->publishPlane(coverage_corners, ns, 0.01, color);

            // Add small delay to allow RViz to process
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        RCLCPP_INFO(logger, "Grid coverage visualization published");
    }
    
    // Generate camera positions by moving along normal direction from plane points
    std::vector<Viewpoint> all_viewpoints;
    
    // Use the initial camera orientation for all viewpoints (in moveit frame)
    // All viewpoints have the same orientation, only position varies
    const Eigen::Quaterniond cam_orientation_base = geometry_utils::arrayToEigenQuat(init_cam_orientation);
    
    // Transform initial orientation to map frame if needed
    Eigen::Quaterniond cam_orientation_map = cam_orientation_base;
    if (moveit_interface->getPoseReferenceFrame() != octomap_interface->getOctomapFrameId()) {
        try {
            geometry_msgs::msg::TransformStamped transform_stamped = 
                tf_buffer->lookupTransform(
                    octomap_interface->getOctomapFrameId(),
                    moveit_interface->getPoseReferenceFrame(),
                    tf2::TimePointZero);
            Eigen::Isometry3d transform = tf2::transformToEigen(transform_stamped.transform);
            Eigen::Quaterniond rotation(transform.rotation());
            cam_orientation_map = rotation * cam_orientation_base;
            cam_orientation_map.normalize();
        } catch (const tf2::TransformException& ex) {
            RCLCPP_ERROR(logger, "TF2 exception during orientation transform: %s", ex.what());
            return {};
        }
    }
    
    for (size_t idx = 0; idx < grid_points_on_plane.size(); ++idx) {
        const auto& plane_point = grid_points_on_plane[idx];
        
        // Camera position is at distance along normal from plane point
        // Place camera on the opposite side (subtract normal direction)
        Eigen::Vector3d cam_position_map = plane_point - distance * normal;
        
        RCLCPP_DEBUG(logger, "Grid point %zu: plane=[%.3f, %.3f, %.3f], camera=[%.3f, %.3f, %.3f]",
                    idx, plane_point.x(), plane_point.y(), plane_point.z(),
                    cam_position_map.x(), cam_position_map.y(), cam_position_map.z());
        
        // Transform camera pose from map frame back to moveit frame
        Eigen::Vector3d cam_position_moveit = cam_position_map;
        Eigen::Quaterniond cam_orientation_moveit = cam_orientation_base;  // Use original orientation
        
        if (moveit_interface->getPoseReferenceFrame() != octomap_interface->getOctomapFrameId()) {
            try {
                geometry_msgs::msg::TransformStamped transform_stamped = 
                    tf_buffer->lookupTransform(
                        moveit_interface->getPoseReferenceFrame(),
                        octomap_interface->getOctomapFrameId(),
                        tf2::TimePointZero);
                
                Eigen::Isometry3d transform = tf2::transformToEigen(transform_stamped.transform);
                
                // Transform position
                cam_position_moveit = transform * cam_position_map;
                
            } catch (const tf2::TransformException& ex) {
                RCLCPP_ERROR(logger, "TF2 exception during viewpoint transform: %s", ex.what());
                return {};
            }
        }
        
        // Create Viewpoint with position and orientation in moveit frame
        Viewpoint vp;
        vp.position = cam_position_moveit;
        vp.orientation = {cam_orientation_moveit.x(), cam_orientation_moveit.y(), 
                          cam_orientation_moveit.z(), cam_orientation_moveit.w()};
        all_viewpoints.push_back(vp);
    }
    
    RCLCPP_INFO(logger, "Generated %zu total viewpoints", all_viewpoints.size());
    
    // Log detailed information about viewpoint positions
    RCLCPP_INFO(logger, "\nViewpoint Details (in map frame):");
    for (size_t i = 0; i < grid_points_on_plane.size() && i < 5; ++i) {
        const Eigen::Vector3d& grid_pt = grid_points_on_plane[i];
        Eigen::Vector3d cam_pt = grid_pt - distance * normal;
        RCLCPP_INFO(logger, "  VP %zu: grid=[%.3f, %.3f, %.3f] -> camera=[%.3f, %.3f, %.3f]",
                   i, grid_pt.x(), grid_pt.y(), grid_pt.z(),
                   cam_pt.x(), cam_pt.y(), cam_pt.z());
    }
    if (grid_points_on_plane.size() > 5) {
        RCLCPP_INFO(logger, "  ... (%zu more viewpoints)", grid_points_on_plane.size() - 5);
    }

    // Visualize viewpoints in the map frame (same frame as grid coverage)
    if (visualizer) {
        RCLCPP_INFO(logger, "\nPublishing all viewpoints for visualization...");
        for (size_t i = 0; i < all_viewpoints.size(); ++i) {
            
            visualizer->publishViewpoint(
                all_viewpoints[i],
                0.2, 0.02, 1.0f,
                "vpoint_" + std::to_string(i),
                moveit_interface->getPoseReferenceFrame());

            // Add small delay to allow RViz to process
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }    
    }

    return all_viewpoints;
}

std::optional<std::pair<Viewpoint, moveit::planning_interface::MoveGroupInterface::Plan>>
selectNextViewpointWithPlan(
    const Viewpoint& viewpoint,
    const std::shared_ptr<MoveItInterface>& interface,
    const std::shared_ptr<NBVVisualizer>& visualizer,
    const NBVPlannerConfig& config,
    const rclcpp::Logger& logger)
{
    RCLCPP_INFO(logger, "\n=== Plan Next Viewpoint ===");
    
    std::vector<double> current_joint_state;
    if (!interface->getCurrentJointAngles(current_joint_state)) {
        RCLCPP_ERROR(logger, "Failed to get current joint state for IK seed");
        return std::nullopt;
    }
        
    // Convert camera pose to end-effector pose
    geometry_msgs::msg::Pose cam_pose = eigenToPose(viewpoint.position, viewpoint.orientation);
    geometry_msgs::msg::Pose target_ee_pose;
    if (!interface->cameraPoseToEEPose(cam_pose, config.camera_optical_link, target_ee_pose)) {
        RCLCPP_DEBUG(logger, "  Failed to convert camera pose to EE pose...");
        return std::nullopt;
    }
    
    // Compute IK
    auto next_joint_angles = interface->computeIK(current_joint_state, target_ee_pose, 0.01, 5);
    if (next_joint_angles.empty()) {
        RCLCPP_INFO(logger, "  No IK solution found for this viewpoint...");
        return std::nullopt;
    }
    
    // Check collision-free
    if (!interface->isStateValid(next_joint_angles)) {
        RCLCPP_INFO(logger, "  IK solution is in collision...");
        return std::nullopt;
    }
    
    // Plan path
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    interface->planToJointGoal(next_joint_angles, plan);
    if (plan.trajectory_.joint_trajectory.points.empty()) {
        RCLCPP_INFO(logger, "  No reachable path to IK solution...");
        return std::nullopt;
    }
    
    RCLCPP_INFO(logger, "Next Viewpoint:");
    RCLCPP_INFO(logger, "  Position: [%.3f, %.3f, %.3f]",
                viewpoint.position.x(), viewpoint.position.y(), viewpoint.position.z());
    if (visualizer) {
        visualizer->publishCoordinate(eigenToPose(viewpoint.position, viewpoint.orientation),
                                        0.15, 0.01, 1.0f, "next_viewpoint");
        RCLCPP_INFO(logger, "Next viewpoint published for visualization");
    }
    
    return std::make_pair(viewpoint, plan);
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
    
    // Save the metrics to file and plot
    std::string csv_path = config.metrics_data_dir + "/nbv_metrics.csv";
    std::string plot_path = config.metrics_plots_dir + "/nbv_metrics_final.png";
    visualizer->logMetricsToCSV(all_metrics, csv_path);
    visualizer->plotMetrics(all_metrics, "NBV Metrics - Final", {}, plot_path);
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
    auto moveit_interface = setupMoveItInterface(node, config.manipulator_group_name);
    RCLCPP_INFO(node->get_logger(), "MoveIt Interface initialized for group: %s with base link: %s",
                config.manipulator_group_name.c_str(), moveit_interface->getPoseReferenceFrame().c_str());

    // Initialize visualizer if requested
    std::shared_ptr<NBVVisualizer> visualizer;
    if (config.visualize) {
        visualizer = std::make_shared<NBVVisualizer>(node, config.map_frame, config.visualization_topic);
        RCLCPP_INFO(node->get_logger(), "Visualizer initialized on topic: %s", config.visualization_topic.c_str());
    }

    // Move to initial joint configuration
    std::vector<double> init_joint_angles_rad = geometry_utils::deg2Rad({0.0, -45.0, -45.0, 0.0, 0.0, 90.0});
    RCLCPP_INFO(node->get_logger(), "\nMoving to initial joint configuration...");
    if (!moveit_interface->planAndExecute(init_joint_angles_rad)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to move to initial joint configuration");
        rclcpp::shutdown();
        return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Moved to initial joint configuration successfully");

    // Get initial camera pose wrt moveit base link
    Eigen::Vector3d init_cam_position;
    std::array<double, 4> init_cam_orientation;
    if (!moveit_interface->getLinkPose(config.camera_optical_link, init_cam_position, init_cam_orientation)) {
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
    auto workspace = setupWorkspace(moveit_interface, visualizer, config, node->get_logger());
    if (!workspace) {
        rclcpp::shutdown();
        return 1;
    }

    // Initialize TF2 buffer and listener
    RCLCPP_INFO(node->get_logger(), "\n=== Initialize TF2 ===");
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    RCLCPP_INFO(node->get_logger(), "TF2 buffer and listener initialized");

    // Initialize octomap interface
    RCLCPP_INFO(node->get_logger(), "\n=== Receive OctoMap ===");
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

    // Get the predefined viewpoint pattern
    auto reachable_viewpoints = generateViewpoints(
        workspace, moveit_interface, octomap_interface, visualizer, tf_buffer,
        init_cam_position, init_cam_orientation, 
        0.2,   // overlap ratio
        true, // use_current_distance (false = use ideal distance from config)
        config, node->get_logger());
    // if (reachable_viewpoints.empty()) {
    //     RCLCPP_WARN(node->get_logger(), "No reachable viewpoints generated!");
    //     rclcpp::shutdown();
    //     return 1;
    // }

    // // Main NBV Planning Loop
    // for (int iter = 0; iter < config.max_iterations; ++iter) {
    //     RCLCPP_INFO(node->get_logger(), "\n********** NBV Baseline Iteration %d **********", iter + 1);

    //     // Select best viewpoint with valid plan
    //     auto best_result = selectNextViewpointWithPlan(
    //         reachable_viewpoints, interface, visualizer, config, node->get_logger());
    //     if (!best_result) {
    //         RCLCPP_WARN(node->get_logger(), "No valid viewpoint with path to IK solution found!");
    //         break;
    //     }

    //     auto [next_viewpoint, plan] = *best_result;

    //     // Execute motion
    //     if (!executeAndWaitForMotion(interface, node, plan, node->get_logger())) {
    //         RCLCPP_ERROR(node->get_logger(), "Motion execution failed, ending NBV planning");
    //         break;
    //     }

    //     // Wait for octomap update
    //     waitForOctomapUpdate(node, octomap_interface, trigger_clients, 
    //                         config.capture_type, node->get_logger());

    //     // Evaluate if enabled
    //     if (config.enable_evaluation && octomap_interface->isSemanticTree()) {
    //         performEvaluation(octomap_interface, visualizer, all_metrics, config, node->get_logger());
    //     }

    //     // Clear visualization
    //     if (visualizer) {
    //         visualizer->clearAllMarkers();
    //         RCLCPP_INFO(node->get_logger(), "Cleared all visualization markers");
    //     }
    // }

    // // Return to initial pose
    // RCLCPP_INFO(node->get_logger(), "\nMoving back to initial joint configuration...");
    // if (!moveToInitialPose(interface, init_joint_angles_rad, node->get_logger())) {
    //     RCLCPP_WARN(node->get_logger(), "Failed to move back to initial joint configuration");
    // } else {
    //     RCLCPP_INFO(node->get_logger(), "Successfully returned to initial joint configuration");
    // }

    RCLCPP_INFO(node->get_logger(), "Press Ctrl+C to exit.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
