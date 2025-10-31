/**
 * @file occupancy_map_visualizer.cpp
 * @brief Implementation of RViz visualization
 */

#include "husky_xarm6_mcr_occupancy_map/occupancy_map_visualizer.hpp"
#include <geometry_msgs/msg/point.hpp>

namespace husky_xarm6_mcr_occupancy_map
{

    OccupancyMapVisualizer::OccupancyMapVisualizer(
        const rclcpp::Node::SharedPtr &node,
        const OccupancyMapTreePtr &tree,
        const std::string &map_frame)
        : node_(node), tree_(tree), map_frame_(map_frame), logger_(node_->get_logger()), last_publish_time_(node_->now()), update_rate_(1.0)
    {
        marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "occupancy_map/markers", 10);

        RCLCPP_INFO(logger_, "OccupancyMapVisualizer created");
    }

    void OccupancyMapVisualizer::setUpdateRate(double rate)
    {
        update_rate_ = rate;
    }

    void OccupancyMapVisualizer::publishMarkers(bool publish_free)
    {
        // Rate limiting
        auto now = node_->now();
        if ((now - last_publish_time_).seconds() < (1.0 / update_rate_))
        {
            return;
        }
        last_publish_time_ = now;

        if (!tree_)
        {
            return;
        }

        // Collect voxel centers
        std::vector<octomap::point3d> occupied_points;
        std::vector<octomap::point3d> free_points;

        tree_->lockRead();

        for (auto it = tree_->begin_leafs(); it != tree_->end_leafs(); ++it)
        {
            octomap::point3d coord = it.getCoordinate();

            if (tree_->isNodeOccupied(*it))
            {
                occupied_points.push_back(coord);
            }
            else if (publish_free)
            {
                free_points.push_back(coord);
            }
        }

        tree_->unlockRead();

        // Create marker array
        visualization_msgs::msg::MarkerArray marker_array;

        // Occupied voxels (blue, opaque)
        if (!occupied_points.empty())
        {
            std_msgs::msg::ColorRGBA occupied_color;
            occupied_color.r = 0.0;
            occupied_color.g = 0.4;
            occupied_color.b = 1.0;
            occupied_color.a = 0.8;

            auto occupied_marker = createVoxelMarker(
                occupied_points,
                0,
                "occupied",
                occupied_color,
                tree_->getResolution());

            marker_array.markers.push_back(occupied_marker);
        }

        // Free voxels (green, transparent) - optional for performance
        if (publish_free && !free_points.empty())
        {
            // Limit number of free voxels to display
            if (free_points.size() > 50000)
            {
                free_points.resize(50000);
            }

            std_msgs::msg::ColorRGBA free_color;
            free_color.r = 0.0;
            free_color.g = 1.0;
            free_color.b = 0.0;
            free_color.a = 0.1;

            auto free_marker = createVoxelMarker(
                free_points,
                1,
                "free",
                free_color,
                tree_->getResolution());

            marker_array.markers.push_back(free_marker);
        }

        // Publish
        if (!marker_array.markers.empty())
        {
            marker_pub_->publish(marker_array);
            RCLCPP_DEBUG(
                logger_,
                "Published markers: %zu occupied, %zu free",
                occupied_points.size(),
                free_points.size());
        }
    }

    visualization_msgs::msg::Marker OccupancyMapVisualizer::createVoxelMarker(
        const std::vector<octomap::point3d> &points,
        int id,
        const std::string &ns,
        const std_msgs::msg::ColorRGBA &color,
        double size)
    {
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = map_frame_;
        marker.header.stamp = node_->now();
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.scale.x = size;
        marker.scale.y = size;
        marker.scale.z = size;

        marker.color = color;

        marker.lifetime = rclcpp::Duration::from_seconds(0); // Persist until next update

        // Add points
        marker.points.reserve(points.size());
        for (const auto &point : points)
        {
            geometry_msgs::msg::Point p;
            p.x = point.x();
            p.y = point.y();
            p.z = point.z();
            marker.points.push_back(p);
        }

        return marker;
    }

} // namespace husky_xarm6_mcr_occupancy_map
