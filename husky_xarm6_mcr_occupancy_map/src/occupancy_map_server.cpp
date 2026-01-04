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

    auto node = std::make_shared<rclcpp::Node>("occupancy_map_server");

    auto logger = node->get_logger();
    RCLCPP_INFO(logger, "Starting occupancy map server...");

    // Create TF buffer and listener
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // Declare all parameters (YAML will override these defaults)
    node->declare_parameter("resolution", 0.1);
    node->declare_parameter("map_frame", "map");
    node->declare_parameter("max_range", 5.0);
    node->declare_parameter("min_range", 0.0);
    node->declare_parameter("prob_hit", 0.7);
    node->declare_parameter("prob_miss", 0.4);
    node->declare_parameter("clamp_min", 0.1192);
    node->declare_parameter("clamp_max", 0.971);
    node->declare_parameter("occupancy_threshold", 0.5);
    node->declare_parameter("filter_ground_plane", true);
    node->declare_parameter("ground_distance_threshold", 0.04);
    node->declare_parameter("pointcloud_topic", "/camera/depth/points");
    node->declare_parameter("publish_free_voxels", false);
    node->declare_parameter("visualization_rate", 1.0);

    // Get parameters (already declared by automatically_declare_parameters_from_overrides)
    OccupancyMapParameters params;
    params.resolution = node->get_parameter("resolution").as_double();
    params.max_range = node->get_parameter("max_range").as_double();
    params.min_range = node->get_parameter("min_range").as_double();
    params.prob_hit = node->get_parameter("prob_hit").as_double();
    params.prob_miss = node->get_parameter("prob_miss").as_double();
    params.clamp_min = node->get_parameter("clamp_min").as_double();
    params.clamp_max = node->get_parameter("clamp_max").as_double();
    params.occupancy_threshold = node->get_parameter("occupancy_threshold").as_double();
    params.map_frame = node->get_parameter("map_frame").as_string();
    params.filter_ground_plane = node->get_parameter("filter_ground_plane").as_bool();
    params.ground_distance_threshold = node->get_parameter("ground_distance_threshold").as_double();

    std::string pointcloud_topic = node->get_parameter("pointcloud_topic").as_string();
    bool publish_free = node->get_parameter("publish_free_voxels").as_bool();
    double viz_rate = node->get_parameter("visualization_rate").as_double();

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
