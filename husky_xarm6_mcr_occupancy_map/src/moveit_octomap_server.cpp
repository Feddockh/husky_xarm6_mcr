/**
 * @file moveit_octomap_server.cpp
 * @brief MoveIt-integrated occupancy map server with octomap_msgs publishing
 * Supports both regular octomap and semantic octomap modes
 */

#include "husky_xarm6_mcr_occupancy_map/occupancy_map_monitor.hpp"
#include "husky_xarm6_mcr_occupancy_map/pointcloud_updater.hpp"
#include "husky_xarm6_mcr_occupancy_map/semantic_occupancy_map_monitor.hpp"
#include "husky_xarm6_mcr_occupancy_map/semantic_pointcloud_updater.hpp"
#include "husky_xarm6_mcr_occupancy_map/occupancy_map_visualizer.hpp"
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
    node->declare_parameter("planning_scene_world_topic", "/planning_scene_world");

    node->declare_parameter("publish_free_voxels", false);
    node->declare_parameter("enable_visualization", true);
    node->declare_parameter("visualization_topic", "occupancy_map_markers");
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

    node->declare_parameter("use_semantic", false);
    node->declare_parameter("num_classes", 100);
    node->declare_parameter("detections_topic", "/detections");

    node->declare_parameter("nbv_octomap_topic", "/octomap_binary");
    node->declare_parameter("nbv_octomap_qos_transient_local", true);

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
    std::string planning_scene_world_topic =
        node->get_parameter("planning_scene_world_topic").as_string();

    bool publish_free = node->get_parameter("publish_free_voxels").as_bool();
    bool enable_viz = node->get_parameter("enable_visualization").as_bool();
    std::string viz_topic = node->get_parameter("visualization_topic").as_string();
    double viz_rate = node->get_parameter("visualization_rate").as_double();
    double octomap_publish_rate = node->get_parameter("octomap_publish_rate").as_double();

    params.use_bounding_box = node->get_parameter("use_bounding_box").as_bool();
    if (params.use_bounding_box)
    {
        params.bbx_min = octomap::point3d(
            node->get_parameter("bbx_min_x").as_double(),
            node->get_parameter("bbx_min_y").as_double(),
            node->get_parameter("bbx_min_z").as_double());
        params.bbx_max = octomap::point3d(
            node->get_parameter("bbx_max_x").as_double(),
            node->get_parameter("bbx_max_y").as_double(),
            node->get_parameter("bbx_max_z").as_double());
    }

    bool use_semantic = node->get_parameter("use_semantic").as_bool();
    int32_t num_classes = node->get_parameter("num_classes").as_int();
    std::string detections_topic = node->get_parameter("detections_topic").as_string();

    std::string nbv_octomap_topic = node->get_parameter("nbv_octomap_topic").as_string();
    bool nbv_transient = node->get_parameter("nbv_octomap_qos_transient_local").as_bool();

    RCLCPP_INFO(logger, "Server mode: %s", use_semantic ? "SEMANTIC" : "STANDARD");

    // TODO: NOt incorperated yet
    use_semantic = false;
    if (use_semantic)
    {
        // // Semantic mode: use semantic octomap monitor
        // auto semantic_monitor = std::make_shared<SemanticOccupancyMapMonitor>(params, num_classes);

        // // Create semantic pointcloud updater
        // auto semantic_updater = std::make_shared<SemanticPointCloudUpdater>(
        //     pointcloud_topic, detections_topic, params);
        // semantic_updater->initialize(node, tf_buffer);
        // semantic_updater->setSemanticMonitor(semantic_monitor);

        // // TODO: Create visualizer compatible with raw OcTree
        // // For now, visualizer is disabled in semantic mode

        // // Octomap publisher for planning scene world updates
        // auto psw_pub = node->create_publisher<moveit_msgs::msg::PlanningSceneWorld>(
        //     planning_scene_world_topic, rclcpp::QoS(1).transient_local());

        // rclcpp::Time last_octomap_publish = node->now();

        // // Connect update callback
        // semantic_monitor->setUpdateCallback([node,
        //                                     psw_pub,
        //                                     &last_octomap_publish,
        //                                     octomap_publish_rate,
        //                                     semantic_monitor,
        //                                     params]()
        // {
        //     auto now = node->now();
        //     if ((now - last_octomap_publish).seconds() >= (1.0 / octomap_publish_rate))
        //     {
        //         octomap_msgs::msg::Octomap octomap_msg;
        //         octomap_msg.header.frame_id = params.map_frame;
        //         octomap_msg.header.stamp = now;

        //         auto tree = semantic_monitor->getMapTree();

        //         if (octomap_msgs::binaryMapToMsg(*tree, octomap_msg))
        //         {
        //             octomap_msgs::msg::OctomapWithPose octomap_with_pose;
        //             octomap_with_pose.header = octomap_msg.header;
        //             octomap_with_pose.octomap = octomap_msg;
        //             octomap_with_pose.origin.orientation.w = 1.0;

        //             moveit_msgs::msg::PlanningSceneWorld world_msg;
        //             world_msg.octomap = octomap_with_pose;
        //             psw_pub->publish(world_msg);
        //         }

        //         last_octomap_publish = now;
        //     }
        // });

        // // Start monitoring
        // semantic_monitor->startMonitor();
        // semantic_updater->start();

        // RCLCPP_INFO(logger, "Semantic MoveIt octomap server ready");
        // RCLCPP_INFO(logger, "  Detections topic: %s", detections_topic.c_str());
        // RCLCPP_INFO(logger, "  Num classes: %d", num_classes);
    }
    else
    {
        // Standard mode: use regular octomap monitor
        auto monitor = std::make_shared<OccupancyMapMonitor>(node, params);

        // Create PointCloud updater
        auto pc_updater = std::make_shared<PointCloudUpdater>(pointcloud_topic, params);
        pc_updater->initialize(node, tf_buffer);
        monitor->addUpdater(pc_updater);

        // Create visualizer with OccupancyMapTree (if enabled)
        std::shared_ptr<OccupancyMapVisualizer> visualizer;
        if (enable_viz)
        {
            visualizer = std::make_shared<OccupancyMapVisualizer>(
                node, monitor->getMapTree(), params.map_frame, viz_topic);
            visualizer->setUpdateRate(viz_rate);
        }

        // Octomap publisher for planning scene world updates
        auto psw_pub = node->create_publisher<moveit_msgs::msg::PlanningSceneWorld>(
            planning_scene_world_topic, rclcpp::QoS(1).transient_local());
        RCLCPP_INFO(node->get_logger(), "Planning scene world publisher created on topic: %s", planning_scene_world_topic.c_str());

        // Create NBV octomap publisher according to user settings
        rclcpp::QoS nbv_qos = rclcpp::QoS(1);
        if (nbv_transient)
        {
            nbv_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
        }
        auto nbv_pub = node->create_publisher<octomap_msgs::msg::Octomap>(
            nbv_octomap_topic, nbv_qos);
        RCLCPP_INFO(node->get_logger(), "NBV octomap publisher created on topic: %s", nbv_octomap_topic.c_str());

        // Last publish time - use shared_ptr so it persists for lambda
        auto last_octomap_publish = std::make_shared<rclcpp::Time>(node->now());
        
        // Connect update callback
        monitor->setUpdateCallback([node,
                                    visualizer,
                                    psw_pub,
                                    nbv_pub,
                                    last_octomap_publish,
                                    octomap_publish_rate,
                                    publish_free,
                                    monitor,
                                    params]()
                                   {
            RCLCPP_INFO(node->get_logger(), "Occupancy map callback triggered");
            
            if (visualizer)
            {
                visualizer->publishMarkers(publish_free);
                RCLCPP_INFO(node->get_logger(), "Occupancy map visualization updated");
            }

            auto now = node->now();
            RCLCPP_INFO(node->get_logger(), "Occupancy map node time: %.3f", now.seconds());
            RCLCPP_INFO(node->get_logger(), "Occupancy map last publish time: %.3f", last_octomap_publish->seconds());
            if ((now - *last_octomap_publish).seconds() >= (1.0 / octomap_publish_rate))
            {
                
                RCLCPP_INFO(node->get_logger(), "Preparing to publish octomap");

                octomap_msgs::msg::Octomap octomap_msg;
                octomap_msg.header.frame_id = params.map_frame;
                octomap_msg.header.stamp = now;

                monitor->getMapTree()->lockRead();
                
                // Convert the octomap to message (binarized)
                if (octomap_msgs::binaryMapToMsg(*monitor->getMapTree(), octomap_msg))
                {
                    // Give the octomap a neutral pose
                    octomap_msgs::msg::OctomapWithPose octomap_with_pose;
                    octomap_with_pose.header = octomap_msg.header;
                    octomap_with_pose.octomap = octomap_msg;
                    octomap_with_pose.origin.orientation.w = 1.0;

                    // Create a planning scene world message and publish
                    moveit_msgs::msg::PlanningSceneWorld world_msg;
                    world_msg.octomap = octomap_with_pose;
                    psw_pub->publish(world_msg);

                    // Send the octomap on the NBV topic as well
                    nbv_pub->publish(octomap_msg);

                    // Print info
                    RCLCPP_INFO(node->get_logger(), "Published octomap with %zu nodes", monitor->getMapTree()->size());
                }
                
                monitor->getMapTree()->unlockRead();
                *last_octomap_publish = now;
            } });

        // Start monitoring
        monitor->startMonitor();

        RCLCPP_INFO(logger, "Standard MoveIt octomap server ready");
    }

    RCLCPP_INFO(logger, "  Map frame: %s", params.map_frame.c_str());
    RCLCPP_INFO(logger, "  Resolution: %.3f m", params.resolution);
    RCLCPP_INFO(logger, "  Max range: %.2f m", params.max_range);
    RCLCPP_INFO(logger, "  PointCloud topic: %s", pointcloud_topic.c_str());

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
