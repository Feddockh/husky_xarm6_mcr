/**
 * @file occupancy_map_server.cpp
 * @brief Standalone occupancy map server node
 */

#include "husky_xarm6_mcr_occupancy_map/occupancy_map_monitor.hpp"
#include "husky_xarm6_mcr_occupancy_map/pointcloud_updater.hpp"
#include "husky_xarm6_mcr_occupancy_map/occupancy_map_visualizer.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace husky_xarm6_mcr_occupancy_map;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>(
        "occupancy_map_server",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    auto logger = node->get_logger();
    RCLCPP_INFO(logger, "Starting occupancy map server...");

    // Create TF buffer and listener
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // Get parameters (already declared by automatically_declare_parameters_from_overrides)
    OccupancyMapParameters params;
    params.resolution = node->get_parameter_or("resolution", 0.05);
    params.max_range = node->get_parameter_or("max_range", 5.0);
    params.min_range = node->get_parameter_or("min_range", 0.3);
    params.prob_hit = node->get_parameter_or("prob_hit", 0.7);
    params.prob_miss = node->get_parameter_or("prob_miss", 0.4);
    params.clamp_min = node->get_parameter_or("clamp_min", 0.12);
    params.clamp_max = node->get_parameter_or("clamp_max", 0.97);
    params.occupancy_threshold = node->get_parameter_or("occupancy_threshold", 0.5);
    params.map_frame = node->get_parameter_or<std::string>("map_frame", "odom");
    params.filter_ground_plane = node->get_parameter_or("filter_ground_plane", true);
    params.ground_distance_threshold = node->get_parameter_or("ground_distance_threshold", 0.04);

    std::string pointcloud_topic = node->get_parameter_or<std::string>(
        "pointcloud_topic", "/firefly_camera/points");

    bool publish_free = node->get_parameter_or("publish_free_voxels", false);
    double viz_rate = node->get_parameter_or("visualization_rate", 2.0);

    // Create occupancy map monitor
    auto monitor = std::make_shared<OccupancyMapMonitor>(node, params);

    // Create PointCloud updater
    auto pc_updater = std::make_shared<PointCloudUpdater>(pointcloud_topic, params);
    pc_updater->initialize(node, tf_buffer);
    monitor->addUpdater(pc_updater);

    // Create visualizer
    auto visualizer = std::make_shared<OccupancyMapVisualizer>(
        node,
        monitor->getMapTree(),
        params.map_frame);

    visualizer->setUpdateRate(viz_rate);

    // Connect update callback to visualizer
    monitor->setUpdateCallback([visualizer, publish_free]()
                               { visualizer->publishMarkers(publish_free); });

    // Start monitoring
    monitor->startMonitor();

    RCLCPP_INFO(logger, "Occupancy map server ready");
    RCLCPP_INFO(logger, "  Map frame: %s", params.map_frame.c_str());
    RCLCPP_INFO(logger, "  Resolution: %.3f m", params.resolution);
    RCLCPP_INFO(logger, "  Max range: %.2f m", params.max_range);
    RCLCPP_INFO(logger, "  PointCloud topic: %s", pointcloud_topic.c_str());

    rclcpp::spin(node);

    // Clean shutdown
    monitor->stopMonitor();
    rclcpp::shutdown();

    return 0;
}
