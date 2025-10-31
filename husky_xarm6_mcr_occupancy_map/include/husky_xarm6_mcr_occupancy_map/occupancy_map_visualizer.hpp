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
         */
        OccupancyMapVisualizer(
            const rclcpp::Node::SharedPtr &node,
            const OccupancyMapTreePtr &tree,
            const std::string &map_frame);

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
