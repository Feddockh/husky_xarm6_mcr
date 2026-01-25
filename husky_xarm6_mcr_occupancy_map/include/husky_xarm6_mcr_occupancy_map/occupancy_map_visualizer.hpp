/**
 * @file occupancy_map_visualizer.hpp
 * @brief RViz visualization for occupancy map
 */

#pragma once

#include "husky_xarm6_mcr_occupancy_map/occupancy_map_tree.hpp"

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <memory>
#include <string>
#include <optional>

namespace husky_xarm6_mcr_occupancy_map
{

    /**
     * @brief Publishes RViz markers for occupancy map visualization
     *
     * Creates marker arrays showing occupied voxels (and optionally free voxels)
     * for debugging and visualization purposes.
     */
    class OccupancyMapVisualizer
    {
    public:
        /**
         * @brief Constructor
         * @param node ROS2 node
         * @param tree Octree to visualize
         * @param map_frame Frame ID for markers
         * @param topic Marker topic to publish on
         */
        OccupancyMapVisualizer(
            const rclcpp::Node::SharedPtr &node,
            const OccupancyMapTreePtr &tree,
            const std::string &map_frame,
            const std::string &topic = "occupancy_map_markers");

        /**
         * @brief Publish marker visualization
         * @param publish_free Whether to publish free voxels (can be slow)
         */
        void publishMarkers(bool publish_free = false);

        /**
         * @brief Set update rate (Hz)
         */
        void setUpdateRate(double rate);

    private:
        /**
         * @brief Publish bounding box marker
         * @param marker_array Marker array to add bbox marker to
         * @param stamp Timestamp for the marker
         * @param bbx_min Optional minimum corner of bounding box (uses tree's if not provided)
         * @param bbx_max Optional maximum corner of bounding box (uses tree's if not provided)
         */
        void publishBoundingBox(
            visualization_msgs::msg::MarkerArray &marker_array,
            const rclcpp::Time &stamp,
            const std::optional<octomap::point3d> &bbx_min = std::nullopt,
            const std::optional<octomap::point3d> &bbx_max = std::nullopt);

        /**
         * @brief Create marker for voxels
         */
        visualization_msgs::msg::Marker createVoxelMarker(
            const std::vector<octomap::point3d> &points,
            int id,
            const std::string &ns,
            const std_msgs::msg::ColorRGBA &color,
            double size);

        rclcpp::Node::SharedPtr node_;
        OccupancyMapTreePtr tree_;
        std::string map_frame_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
        rclcpp::Logger logger_;
        rclcpp::Time last_publish_time_;
        double update_rate_;
    };

    using OccupancyMapVisualizerPtr = std::shared_ptr<OccupancyMapVisualizer>;

} // namespace husky_xarm6_mcr_occupancy_map
