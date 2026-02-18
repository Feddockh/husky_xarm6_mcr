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

using namespace husky_xarm6_mcr_nbv_planner;
using namespace husky_xarm6_mcr_nbv_planner::conversions;

// ============================================================================
// Configuration Structure
// ============================================================================
struct NBVPlannerConfig
{
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
    double camera_horizontal_fov; // radians
    double camera_vertical_fov; // radians
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

struct TriggerClients
{
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

NBVPlannerConfig loadConfiguration(const std::shared_ptr<rclcpp::Node> &node)
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
    config.camera_horizontal_fov = node->get_parameter("camera_horizontal_fov").as_double() * M_PI / 180.0; // Convert degrees to radians
    config.camera_vertical_fov = node->get_parameter("camera_vertical_fov").as_double() * M_PI / 180.0;     // Convert degrees to radians
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

void printConfiguration(const NBVPlannerConfig &config, const rclcpp::Logger &logger)
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
    const std::shared_ptr<rclcpp::Node> &node,
    const std::string &manipulator_group_name)
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
    const std::shared_ptr<MoveItInterface> &interface,
    const std::shared_ptr<NBVVisualizer> &visualizer,
    const NBVPlannerConfig &config,
    const rclcpp::Logger &logger)
{
    RCLCPP_INFO(logger, "\n=== Manipulation Workspace ===");
    RCLCPP_INFO(logger, "Initializing Manipulation Workspace...");

    auto workspace = std::make_shared<ManipulationWorkspace>(interface, visualizer);

    if (config.learn_workspace)
    {
        RCLCPP_INFO(logger, "Learning workspace with %d samples...", config.num_samples);
        if (!workspace->learnWorkspace(config.num_samples, config.visualize))
        {
            RCLCPP_ERROR(logger, "Failed to learn workspace");
            return nullptr;
        }
        RCLCPP_INFO(logger, "Learned %zu voxels, saving to: %s",
                    workspace->getNumReachableVoxels(), config.workspace_file.c_str());
        workspace->saveWorkspaceToFile(config.workspace_file);
    }
    else
    {
        RCLCPP_INFO(logger, "Loading workspace from: %s", config.workspace_file.c_str());
        if (!workspace->loadWorkspaceFromFile(config.workspace_file))
        {
            RCLCPP_ERROR(logger, "Failed to load. Set learn_workspace:=true");
            return nullptr;
        }
        RCLCPP_INFO(logger, "Loaded %zu voxels", workspace->getNumReachableVoxels());
    }

    return workspace;
}

TriggerClients createTriggerClients(const std::shared_ptr<rclcpp::Node> &node)
{
    return {
        node->create_client<std_srvs::srv::Trigger>("/trigger/start_video"),
        node->create_client<std_srvs::srv::Trigger>("/trigger/stop_video"),
        node->create_client<std_srvs::srv::Trigger>("/trigger/send_trigger")};
}

bool startContinuousCapture(
    const std::shared_ptr<rclcpp::Node> &node,
    const TriggerClients &clients,
    const rclcpp::Logger &logger)
{
    RCLCPP_INFO(logger, "Starting continuous video capture...");
    if (!clients.start_video->wait_for_service(std::chrono::seconds(5)))
    {
        RCLCPP_WARN(logger, "start_video service not available, continuing anyway...");
        return false;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = clients.start_video->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(5)) == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = future.get();
        if (response->success)
        {
            RCLCPP_INFO(logger, "Continuous video capture started: %s", response->message.c_str());
            return true;
        }
        else
        {
            RCLCPP_WARN(logger, "Failed to start video: %s", response->message.c_str());
        }
    }
    return false;
}

bool stopVideoCapture(
    const std::shared_ptr<rclcpp::Node> &node,
    const TriggerClients &clients,
    const rclcpp::Logger &logger)
{
    RCLCPP_INFO(logger, "Using triggered capture mode, ensuring video is stopped...");
    if (!clients.stop_video->wait_for_service(std::chrono::seconds(5)))
    {
        RCLCPP_WARN(logger, "stop_video service not available, continuing anyway...");
        return false;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = clients.stop_video->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(5)) == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = future.get();
        RCLCPP_INFO(logger, "Stop video service called: %s", response->message.c_str());
        return true;
    }
    return false;
}

bool waitForOctomapWithTriggers(
    const std::shared_ptr<rclcpp::Node> &node,
    const std::shared_ptr<OctoMapInterface> &octomap_interface,
    const TriggerClients &clients,
    const std::string &capture_type,
    const std::string &octomap_topic,
    double timeout_seconds,
    const rclcpp::Logger &logger)
{
    RCLCPP_INFO(logger, "Waiting for octomap to be published on %s...", octomap_topic.c_str());

    rclcpp::Time start_time = node->now();
    rclcpp::Rate spin_rate(10);

    if (capture_type == "triggered")
    {
        // Triggered mode: send triggers periodically until octomap is available
        while (rclcpp::ok() && !octomap_interface->isTreeAvailable() &&
               (node->now() - start_time).seconds() < timeout_seconds)
        {
            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
            auto future = clients.send_trigger->async_send_request(request);
            RCLCPP_INFO(logger, "Sending camera trigger...");

            // Wait 1 second for trigger response and octomap
            auto wait_start = node->now();
            while (rclcpp::ok() && (node->now() - wait_start).seconds() < 1.0)
            {
                rclcpp::spin_some(node);
                if (octomap_interface->isTreeAvailable())
                    break;
                spin_rate.sleep();
            }
        }
    }
    else
    {
        // Continuous mode: just wait for octomap
        while (rclcpp::ok() && !octomap_interface->isTreeAvailable() &&
               (node->now() - start_time).seconds() < timeout_seconds)
        {
            rclcpp::spin_some(node);
            spin_rate.sleep();
        }
    }

    if (!octomap_interface->isTreeAvailable())
    {
        RCLCPP_ERROR(logger, "Octomap not available after %.0f seconds. Exiting.", timeout_seconds);
        return false;
    }
    RCLCPP_INFO(logger, "Octomap received successfully");
    return true;
}

/**
 * @brief Get transform from MoveIt planning frame to OctoMap frame.
 * Returns identity transform if frames are the same.
 */
geometry_msgs::msg::TransformStamped getMoveitToOctomapTransform(
    const std::shared_ptr<MoveItInterface>& moveit_interface,
    const std::shared_ptr<OctoMapInterface>& octomap_interface,
    const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
    const rclcpp::Logger& logger)
{
    std::string moveit_frame = moveit_interface->getPoseReferenceFrame();
    std::string octomap_frame = octomap_interface->getOctomapFrameId();
    
    geometry_msgs::msg::TransformStamped transform;
    
    if (moveit_frame != octomap_frame)
    {
        try
        {
            transform = tf_buffer->lookupTransform(
                octomap_frame,
                moveit_frame,
                tf2::TimePointZero);
            RCLCPP_INFO(logger, "Transform from %s to %s found",
                        moveit_frame.c_str(), octomap_frame.c_str());
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(logger, "Failed to get transform from %s to %s: %s",
                         moveit_frame.c_str(), octomap_frame.c_str(), ex.what());
            throw;
        }
    }
    else
    {
        RCLCPP_INFO(logger, "MoveIt frame and OctoMap frame are the same: %s", moveit_frame.c_str());
        // Create identity transform
        transform.header.frame_id = moveit_frame;
        transform.child_frame_id = octomap_frame;
        transform.transform.translation.x = 0.0;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        transform.transform.rotation.w = 1.0;
    }
    
    return transform;
}

/**
 * @brief Compute the plane intersection with the bounding box and return the corners and the distance to the midpoint.
 * @init_cam_position: Initial camera position in map frame.
 */
std::pair<std::vector<Eigen::Vector3d>, Eigen::Vector3d> computePlane(
    const std::shared_ptr<OctoMapInterface> &octomap_interface,
    const Eigen::Vector3d &cam_position)
{
    // Compute the midpoint of the bounding box
    octomap::point3d min_octo, max_octo;
    octomap_interface->getBoundingBox(min_octo, max_octo);
    Eigen::Vector3d min_bbox = octomapToEigen(min_octo);
    Eigen::Vector3d max_bbox = octomapToEigen(max_octo);
    Eigen::Vector3d mid = 0.5 * (min_bbox + max_bbox);

    // Compute the viewing direction
    Eigen::Vector3d distance = (cam_position - mid);
    Eigen::Vector3d view_dir = distance.normalized();
    // If the viewing direction is aligned with the x axis, then we are using the YZ plane
    Eigen::Vector3d abs_view_dir = view_dir.cwiseAbs();
    if (abs_view_dir.x() >= abs_view_dir.y()) // YZ plane
    {
        std::vector<Eigen::Vector3d> corners = {
            {mid.x(), min_bbox.y(), min_bbox.z()},
            {mid.x(), max_bbox.y(), min_bbox.z()},
            {mid.x(), max_bbox.y(), max_bbox.z()},
            {mid.x(), min_bbox.y(), max_bbox.z()},
        };
        return std::make_pair(corners, distance);
    }
    else // XZ plane
    {
        std::vector<Eigen::Vector3d> corners = {
            {min_bbox.x(), mid.y(), min_bbox.z()},
            {max_bbox.x(), mid.y(), min_bbox.z()},
            {max_bbox.x(), mid.y(), max_bbox.z()},
            {min_bbox.x(), mid.y(), max_bbox.z()},
        };
        return std::make_pair(corners, distance);
    }
}

/**
 * @brief Generate viewpoints facing the plane defined by the corners and distance.
 * Returns the viewpoints and their corresponding coverage planes.
 * @orientation_base: Base orientation for the viewpoints in the map frame.
 */
std::pair<std::vector<Viewpoint>, std::vector<std::vector<Eigen::Vector3d>>> generateViewpointsFromPlane(
    const std::vector<Eigen::Vector3d> &corners,
    const Eigen::Vector3d &distance,
    const Eigen::Quaterniond &orientation_base,
    double overlap_ratio,
    const NBVPlannerConfig &config,
    const rclcpp::Logger &logger)
{
    // Determine the midpoint of the plane
    Eigen::Vector3d mid = 0.25 * (corners[0] + corners[1] + corners[2] + corners[3]);

    // Determine which plane we are using (YZ or XZ) based on the corners
    bool is_yz_plane = (std::abs(corners[0].x() - corners[1].x()) < 1e-6);

    // Calculate the coverage area at the given distance (just along the x or y direction)
    double u_coverage, v_coverage;
    if (is_yz_plane) // YZ plane
    {
        u_coverage = 2.0 * distance.x() * std::tan(config.camera_horizontal_fov / 2.0);
        v_coverage = 2.0 * distance.x() * std::tan(config.camera_vertical_fov / 2.0);
    }
    else // XZ plane
    {
        u_coverage = 2.0 * distance.y() * std::tan(config.camera_horizontal_fov / 2.0);
        v_coverage = 2.0 * distance.y() * std::tan(config.camera_vertical_fov / 2.0);
    }

    // Once we have the coverage, we can generate a grid of viewpoints on the plane
    double spacing_u = u_coverage * (1.0 - overlap_ratio);
    double spacing_v = v_coverage * (1.0 - overlap_ratio);

    // Starting from the center of the plane, generate grid points
    std::vector<Eigen::Vector3d> viewpoint_positions;
    std::vector<std::vector<Eigen::Vector3d>> coverage_planes;
    int num_u = 0, num_v = 0;
    if (is_yz_plane)
    {
        // YZ plane at constant x
        double plane_height = corners[2].z() - corners[0].z();
        double plane_width = corners[1].y() - corners[0].y();
        // Compute number of viewpoints along each axis
        num_u = std::max(1, static_cast<int>(std::ceil(plane_width / spacing_u)));
        num_v = std::max(1, static_cast<int>(std::ceil(plane_height / spacing_v)));
        // Generate grid points
        double start_y = mid.y() - (num_u - 1) * spacing_u / 2.0;
        double start_z = mid.z() - (num_v - 1) * spacing_v / 2.0;
        for (int i = 0; i < num_u; ++i)
        {
            for (int j = 0; j < num_v; ++j)
            {
                viewpoint_positions.push_back(
                    Eigen::Vector3d(distance.x() + mid.x(), // x is from distance
                                    start_y + i * spacing_u,
                                    start_z + j * spacing_v));
                coverage_planes.push_back({
                    Eigen::Vector3d(mid.x(),
                                    start_y + i * spacing_u - u_coverage / 2.0,
                                    start_z + j * spacing_v - v_coverage / 2.0),
                    Eigen::Vector3d(mid.x(),
                                    start_y + i * spacing_u + u_coverage / 2.0,
                                    start_z + j * spacing_v - v_coverage / 2.0),
                    Eigen::Vector3d(mid.x(),
                                    start_y + i * spacing_u + u_coverage / 2.0,
                                    start_z + j * spacing_v + v_coverage / 2.0),
                    Eigen::Vector3d(mid.x(),
                                    start_y + i * spacing_u - u_coverage / 2.0,
                                    start_z + j * spacing_v + v_coverage / 2.0),
                });
            }
        }
    }
    else // XZ plane
    {
        double plane_height = corners[2].z() - corners[0].z();
        double plane_width = corners[1].x() - corners[0].x();
        num_u = std::max(1, static_cast<int>(std::ceil(plane_width / spacing_u)));
        num_v = std::max(1, static_cast<int>(std::ceil(plane_height / spacing_v)));
        double start_x = mid.x() - (num_u - 1) * spacing_u / 2.0;
        double start_z = mid.z() - (num_v - 1) * spacing_v / 2.0;
        for (int i = 0; i < num_u; ++i)
        {
            for (int j = 0; j < num_v; ++j)
            {
                viewpoint_positions.push_back(
                    Eigen::Vector3d(start_x + i * spacing_u,
                                    distance.y() + mid.y(), // y is from distance
                                    start_z + j * spacing_v));
                coverage_planes.push_back({
                    Eigen::Vector3d(start_x + i * spacing_u - u_coverage / 2.0,
                                    mid.y(),
                                    start_z + j * spacing_v - v_coverage / 2.0),
                    Eigen::Vector3d(start_x + i * spacing_u + u_coverage / 2.0,
                                    mid.y(),
                                    start_z + j * spacing_v - v_coverage / 2.0),
                    Eigen::Vector3d(start_x + i * spacing_u + u_coverage / 2.0,
                                    mid.y(),
                                    start_z + j * spacing_v + v_coverage / 2.0),
                    Eigen::Vector3d(start_x + i * spacing_u - u_coverage / 2.0,
                                    mid.y(),
                                    start_z + j * spacing_v + v_coverage / 2.0),
                });
            }
        }
    }

    // Create the viewpoints with the computed positions and base orientation
    std::vector<Viewpoint> viewpoints;
    for (const auto &pos : viewpoint_positions)
    {
        Viewpoint vp;
        vp.position = pos;
        vp.orientation = {orientation_base.x(), orientation_base.y(),
                          orientation_base.z(), orientation_base.w()};
        viewpoints.push_back(vp);
    }

    RCLCPP_INFO(logger, "Generated %zu viewpoints on the plane (%dx%d grid)", viewpoints.size(), num_u, num_v);
    return std::make_pair(viewpoints, coverage_planes);
}

/**
 * @brief Filter viewpoints to retain only those that are reachable by the manipulator.
 * @viewpoints: Candidate eef poses in moveit frame
 */
std::vector<Viewpoint> filterReachableViewpoints(
    const std::vector<Viewpoint>& viewpoints,
    const std::shared_ptr<ManipulationWorkspace>& manip_workspace,
    const std::shared_ptr<MoveItInterface>& moveit_interface,
    const NBVPlannerConfig& config,
    const rclcpp::Logger& logger)
{    
    // Filter by reachability
    std::vector<Viewpoint> reachable_viewpoints;
    for (const auto& vp : viewpoints) {
        // Convert to end-effector pose
        geometry_msgs::msg::Pose cam_pose = eigenToPose(vp.position, vp.orientation);
        geometry_msgs::msg::Pose ee_pose;
        if (!moveit_interface->cameraPoseToEEPose(cam_pose, config.camera_optical_link, ee_pose)) {
            RCLCPP_INFO(logger, "Failed to convert camera pose to EE pose for reachability check...");
            continue;
        }
        if (manip_workspace->isPoseReachable(ee_pose)) {
            reachable_viewpoints.push_back(vp);
        }
    }
    RCLCPP_INFO(logger, "%zu out of %zu viewpoints are reachable",
                reachable_viewpoints.size(), viewpoints.size());
    
    return reachable_viewpoints;
}

/**
 * @brief Sort the viewpoints based on coordinate values
 * @viewpoints: Candidate viewpoints
 * @axis_priority: Vector indicating axis priority for sorting (e.g., {"x","y","z"} or {"-y","x"} for descending priority on y)
 */
void sortViewpointsByAxisPriority(
    std::vector<Viewpoint> &viewpoints,
    const std::vector<std::string> &axis_priority)
{
    std::sort(viewpoints.begin(), viewpoints.end(),
        [&axis_priority](const Viewpoint& a, const Viewpoint& b) {
            for (const auto& axis_str : axis_priority) {
                // Skip invalid axis (empty string)
                if (axis_str.empty()) {
                    continue;
                }

                // Determine axis and sort order
                double val_a, val_b;
                if (axis_str == "-x") {
                    val_a = -a.position.x();
                    val_b = -b.position.x();
                } else if (axis_str == "x") {
                    val_a = a.position.x();
                    val_b = b.position.x();
                } else if (axis_str == "-y") {
                    val_a = -a.position.y();
                    val_b = -b.position.y();
                } else if (axis_str == "y") {
                    val_a = a.position.y();
                    val_b = b.position.y();
                } else if (axis_str == "-z") {
                    val_a = -a.position.z();
                    val_b = -b.position.z();
                } else if (axis_str == "z") {
                    val_a = a.position.z();
                    val_b = b.position.z();
                } else {
                    continue; // Invalid axis, skip
                }

                // Check if values are different (with small epsilon for floating point)
                if (std::abs(val_a - val_b) > 1e-6)
                    return val_a < val_b;
            }
            return false; // All axes equal
        });
}

std::optional<moveit::planning_interface::MoveGroupInterface::Plan> planPathToViewpoint(
    const Viewpoint &viewpoint,
    const std::shared_ptr<MoveItInterface> &moveit_interface,
    const NBVPlannerConfig &config,
    const rclcpp::Logger &logger)
{
    std::vector<double> current_joint_state;
    if (!moveit_interface->getCurrentJointAngles(current_joint_state))
    {
        RCLCPP_ERROR(logger, "Failed to get current joint state for IK seed");
        return std::nullopt;
    }

    // Convert camera pose to end-effector pose
    geometry_msgs::msg::Pose cam_pose = eigenToPose(viewpoint.position, viewpoint.orientation);
    geometry_msgs::msg::Pose target_ee_pose;
    if (!moveit_interface->cameraPoseToEEPose(cam_pose, config.camera_optical_link, target_ee_pose))
    {
        RCLCPP_INFO(logger, "Failed to convert camera pose to EE pose...");
        return std::nullopt;
    }

    // Compute IK
    auto next_joint_angles = moveit_interface->computeIK(current_joint_state, target_ee_pose, 0.01, 10);
    if (next_joint_angles.empty())
    {
        RCLCPP_INFO(logger, "No IK solution found for this viewpoint...");
        return std::nullopt;
    }

    // Check collision-free
    if (!moveit_interface->isStateValid(next_joint_angles))
    {
        RCLCPP_INFO(logger, "IK solution is in collision...");
        return std::nullopt;
    }

    // Plan path
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit_interface->planToJointGoal(next_joint_angles, plan);
    if (plan.trajectory_.joint_trajectory.points.empty())
    {
        RCLCPP_INFO(logger, "No reachable path to IK solution...");
        return std::nullopt;
    }

    return plan;
}

bool executeAndWaitForMotion(
    const std::shared_ptr<MoveItInterface> &moveit_interface,
    const std::shared_ptr<rclcpp::Node> &node,
    const moveit::planning_interface::MoveGroupInterface::Plan &plan,
    const rclcpp::Logger &logger)
{
    RCLCPP_INFO(logger, "\n=== Step 9: Move to the Best Viewpoint ===");
    RCLCPP_INFO(logger, "Executing motion to best viewpoint...");

    if (!moveit_interface->execute(plan))
    {
        RCLCPP_ERROR(logger, "Failed to execute motion to best viewpoint!");
        return false;
    }

    // Extract target joint angles from plan
    std::vector<double> target_joint_angles = plan.trajectory_.joint_trajectory.points.back().positions;

    // Wait for motion to complete
    rclcpp::Rate spin_rate(10);
    RCLCPP_INFO(logger, "Waiting for manipulator to reach goal...");
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);

        std::vector<double> current_joints;
        if (moveit_interface->getCurrentJointAngles(current_joints))
        {
            double joint_error = 0.0;
            for (size_t i = 0; i < current_joints.size() && i < target_joint_angles.size(); ++i)
            {
                joint_error += std::abs(current_joints[i] - target_joint_angles[i]);
            }
            if (joint_error < 0.01)
            {
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
    const std::shared_ptr<rclcpp::Node> &node,
    const std::shared_ptr<OctoMapInterface> &octomap_interface,
    const TriggerClients &clients,
    const std::string &capture_type,
    const rclcpp::Logger &logger)
{
    bool octomap_updated = false;
    rclcpp::Time initial_octomap_time = octomap_interface->getLastUpdateTime();
    double initial_time_seconds = initial_octomap_time.seconds();
    RCLCPP_INFO(logger, "Initial octomap timestamp: %.3f seconds", initial_time_seconds);

    rclcpp::Rate spin_rate(10);

    if (capture_type == "triggered")
    {
        // Triggered mode: send triggers until octomap updates
        RCLCPP_INFO(logger, "Sending camera triggers until octomap updates...");
        int trigger_count = 0;
        while (rclcpp::ok() && !octomap_updated)
        {
            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
            auto future = clients.send_trigger->async_send_request(request);
            trigger_count++;
            RCLCPP_INFO(logger, "Sending camera trigger #%d...", trigger_count);

            // Wait ~1 second and check for octomap update
            auto wait_start = node->now();
            while (rclcpp::ok() && (node->now() - wait_start).seconds() < 1.0)
            {
                rclcpp::spin_some(node);
                rclcpp::Time new_octomap_time = octomap_interface->getLastUpdateTime();
                double new_time_seconds = new_octomap_time.seconds();

                if (new_time_seconds != initial_time_seconds)
                {
                    octomap_updated = true;
                    RCLCPP_INFO(logger, "Octomap has been updated! New timestamp: %.3f seconds (delta: %.3f)",
                                new_time_seconds, new_time_seconds - initial_time_seconds);
                    break;
                }
                spin_rate.sleep();
            }

            if (!octomap_updated)
            {
                RCLCPP_INFO(logger, "No octomap update detected yet, will send another trigger...");
            }
        }
    }
    else
    {
        // Continuous mode: just wait for octomap update
        while (rclcpp::ok() && !octomap_updated)
        {
            rclcpp::spin_some(node);
            rclcpp::Time new_octomap_time = octomap_interface->getLastUpdateTime();
            double new_time_seconds = new_octomap_time.seconds();

            if (new_time_seconds != initial_time_seconds)
            {
                octomap_updated = true;
                RCLCPP_INFO(logger, "Octomap has been updated with new sensor data");
            }
            spin_rate.sleep();
        }
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
    if (node->get_parameter("use_sim_time").as_bool())
    {
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
    if (config.visualize)
    {
        visualizer = std::make_shared<NBVVisualizer>(node, config.map_frame, config.visualization_topic);
        RCLCPP_INFO(node->get_logger(), "Visualizer initialized on topic: %s", config.visualization_topic.c_str());
    }

    // Move to initial joint configuration
    std::vector<double> init_joint_angles_rad = geometry_utils::deg2Rad({0.0, -45.0, -45.0, 0.0, 0.0, 90.0});
    RCLCPP_INFO(node->get_logger(), "\nMoving to initial joint configuration...");
    if (!moveit_interface->planAndExecute(init_joint_angles_rad))
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to move to initial joint configuration");
        rclcpp::shutdown();
        return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Moved to initial joint configuration successfully");

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
    if (config.capture_type == "continuous")
    {
        startContinuousCapture(node, trigger_clients, node->get_logger());
    }
    else if (config.capture_type == "triggered")
    {
        stopVideoCapture(node, trigger_clients, node->get_logger());
        if (!trigger_clients.send_trigger->wait_for_service(std::chrono::seconds(5)))
        {
            RCLCPP_WARN(node->get_logger(), "send_trigger service not available, waiting for octomap anyway...");
        }
    }

    // Wait for initial octomap
    if (!waitForOctomapWithTriggers(node, octomap_interface, trigger_clients,
                                    config.capture_type, config.octomap_topic,
                                    30.0, node->get_logger()))
    {
        rclcpp::shutdown();
        return 1;
    }

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
        RCLCPP_INFO(node->get_logger(), "Publishing NBV midplane for visualization");
        std_msgs::msg::ColorRGBA plane_color;
        plane_color.r = 0.0f; plane_color.g = 1.0f; plane_color.b = 0.0f; plane_color.a = 0.5f;
        visualizer->publishPlane(plane_corners_map, "nbv_plane", 0.02, plane_color);
    }
    // Generate viewpoints on the plane
    Eigen::Isometry3d transform_eigen = tf2::transformToEigen(moveit_to_octomap_transform.transform);
    Eigen::Quaterniond init_cam_quat_map = Eigen::Quaterniond(transform_eigen.rotation()) * geometry_utils::arrayToEigenQuat(init_cam_orientation);
    double overlap_ratio = 0.45; // 45% overlap
    auto [all_viewpoints_map, coverage_planes_map] = generateViewpointsFromPlane(
        plane_corners_map, distance, init_cam_quat_map, overlap_ratio,
        config, node->get_logger());
    if (config.visualize && visualizer)
    {
        RCLCPP_INFO(node->get_logger(), "Publishing coverage planes for visualization");
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
    std::vector<std::string> axis_priority = {"-z", "-x"};
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
    for (size_t i = 1; i < reachable_viewpoints.size(); i++) {
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
        waitForOctomapUpdate(node, octomap_interface, trigger_clients, config.capture_type, node->get_logger());

        // Evaluate if enabled
        if (config.enable_evaluation && octomap_interface->isSemanticTree()) {
            auto latest_clusters = octomap_interface->clusterSemanticVoxels(false);
            auto match_result = octomap_interface->matchClustersToGroundTruth(latest_clusters, config.eval_threshold_radius, false);
            all_metrics.push_back(octomap_interface->evaluateMatchResults(match_result, false));

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
    }

    // Check if there was an error in the loop
    if (!rclcpp::ok())
    {
        RCLCPP_INFO(node->get_logger(), "\nNBV planning interrupted by shutdown signal, exiting...");
        rclcpp::shutdown();
        return 1;
    }

    // Return to initial joint configuration
    RCLCPP_INFO(node->get_logger(), "\nReturning to initial joint configuration...");
    if (!moveit_interface->planAndExecute(init_joint_angles_rad))
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
