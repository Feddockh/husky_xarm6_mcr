/**
 * @file moveit_octomap_server.cpp
 * @brief MoveIt-integrated occupancy map server with octomap_msgs publishing
 */

#include "husky_xarm6_mcr_occupancy_map/occupancy_map_monitor.hpp"
#include "husky_xarm6_mcr_occupancy_map/pointcloud_updater.hpp"
#include "husky_xarm6_mcr_occupancy_map/occupancy_map_visualizer.hpp"
#include "husky_xarm6_mcr_occupancy_map/voxel_grid_publisher.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <moveit_msgs/msg/planning_scene_world.hpp>
#include <octomap_msgs/msg/octomap_with_pose.hpp>


using namespace husky_xarm6_mcr_occupancy_map;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("moveit_octomap_server");

    auto logger = node->get_logger();
    RCLCPP_INFO(logger, "Starting MoveIt octomap server...");

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
    node->declare_parameter("voxel_grid_topic", "/voxel_grid");
    node->declare_parameter("planning_scene_world_topic", "/planning_scene_world");

    node->declare_parameter("publish_free_voxels", false);
    node->declare_parameter("visualization_rate", 1.0);
    node->declare_parameter("octomap_publish_rate", 1.0);
    // Bounding box parameters
    node->declare_parameter("use_bounding_box", false);
    node->declare_parameter("bbx_min_x", -1.0);
    node->declare_parameter("bbx_min_y", -1.0);
    node->declare_parameter("bbx_min_z", -1.0);
    node->declare_parameter("bbx_max_x", 1.0);
    node->declare_parameter("bbx_max_y", 1.0);
    node->declare_parameter("bbx_max_z", 1.0);

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
    std::string voxel_grid_topic = node->get_parameter("voxel_grid_topic").as_string();
    std::string planning_scene_world_topic =
        node->get_parameter("planning_scene_world_topic").as_string();

    bool publish_free = node->get_parameter("publish_free_voxels").as_bool();
    double viz_rate = node->get_parameter("visualization_rate").as_double();
    double octomap_publish_rate = node->get_parameter("octomap_publish_rate").as_double();

    params.use_bounding_box = node->get_parameter("use_bounding_box").as_bool();
    if (params.use_bounding_box) {
        params.bbx_min = octomap::point3d(
            node->get_parameter("bbx_min_x").as_double(),
            node->get_parameter("bbx_min_y").as_double(),
            node->get_parameter("bbx_min_z").as_double()
        );
        params.bbx_max = octomap::point3d(
            node->get_parameter("bbx_max_x").as_double(),
            node->get_parameter("bbx_max_y").as_double(),
            node->get_parameter("bbx_max_z").as_double()
        );
    }

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

    // Create voxel grid publisher for NBV planner
    auto voxel_grid_pub = std::make_shared<VoxelGridPublisher>(node, voxel_grid_topic);

    // Octomap publisher for planning scene world updates
    auto psw_pub = node->create_publisher<moveit_msgs::msg::PlanningSceneWorld>(
        planning_scene_world_topic, rclcpp::QoS(1).transient_local());

    rclcpp::Time last_octomap_publish = node->now();

    // Connect update callback to visualizer AND octomap publisher
    monitor->setUpdateCallback([node,
                                visualizer,
                                voxel_grid_pub,
                                psw_pub,
                                &last_octomap_publish,
                                octomap_publish_rate,
                                publish_free,
                                monitor,
                                params]()
                               {
    // Publish visualization markers
    visualizer->publishMarkers(publish_free);

    // Rate-limited octomap publishing
    auto now = node->now();
    if ((now - last_octomap_publish).seconds() >= (1.0 / octomap_publish_rate))
    {
        // Serialize octree to binary message
        octomap_msgs::msg::Octomap octomap_msg;
        octomap_msg.header.frame_id = params.map_frame;
        octomap_msg.header.stamp = now;

        monitor->getMapTree()->lockRead();
        
        // Convert OcTree to message (binary format)
        if (octomap_msgs::binaryMapToMsg(*monitor->getMapTree(), octomap_msg))
        {
            // Wrap into OctomapWithPose
            octomap_msgs::msg::OctomapWithPose octomap_with_pose;
            octomap_with_pose.header = octomap_msg.header;
            octomap_with_pose.octomap = octomap_msg;

            // origin pose: identity means "map is expressed in frame_id directly"
            octomap_with_pose.origin.position.x = 0.0;
            octomap_with_pose.origin.position.y = 0.0;
            octomap_with_pose.origin.position.z = 0.0;
            octomap_with_pose.origin.orientation.x = 0.0;
            octomap_with_pose.origin.orientation.y = 0.0;
            octomap_with_pose.origin.orientation.z = 0.0;
            octomap_with_pose.origin.orientation.w = 1.0;

            // NEW: publish to planning_scene_world
            moveit_msgs::msg::PlanningSceneWorld world_msg;
            world_msg.octomap = octomap_with_pose;

            psw_pub->publish(world_msg);
        }
        else
        {
            RCLCPP_WARN(node->get_logger(), "Failed to serialize octomap");
        }

        // Also publish simple voxel grid for NBV planner
        voxel_grid_pub->publishVoxelGrid(monitor->getMapTree(), params.map_frame);

        monitor->getMapTree()->unlockRead();

        last_octomap_publish = now;
    }});

    // Start monitoring
    monitor->startMonitor();

    RCLCPP_INFO(logger, "MoveIt octomap server ready");
    RCLCPP_INFO(logger, "  Map frame: %s", params.map_frame.c_str());
    RCLCPP_INFO(logger, "  Resolution: %.3f m", params.resolution);
    RCLCPP_INFO(logger, "  Max range: %.2f m", params.max_range);
    RCLCPP_INFO(logger, "  PointCloud topic: %s", pointcloud_topic.c_str());
    RCLCPP_INFO(logger, "  Voxel grid topic: %s", voxel_grid_topic.c_str());

    rclcpp::spin(node);

    // Clean shutdown
    monitor->stopMonitor();
    rclcpp::shutdown();

    return 0;
}
