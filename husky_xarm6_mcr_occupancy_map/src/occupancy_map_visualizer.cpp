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
        const std::string &map_frame,
        const std::string &topic)
        : node_(node), tree_(tree), map_frame_(map_frame), logger_(node_->get_logger()),
          last_publish_time_(node_->now()), update_rate_(1.0)
    {
        // Create marker publisher
        marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            topic, 10);

        RCLCPP_INFO(logger_, "OccupancyMapVisualizer created on topic: %s", topic.c_str());
    }

    void OccupancyMapVisualizer::setUpdateRate(double rate)
    {
        update_rate_ = rate;
    }

    void OccupancyMapVisualizer::publishMarkers(bool publish_free)
    {
        // Rate limiting
        auto now = node_->now();
        if ((now - last_publish_time_).seconds() < 1.0 / update_rate_)
        {
            return;
        }
        last_publish_time_ = now;

        if (!tree_)
        {
            return;
        }

        visualization_msgs::msg::MarkerArray marker_array;

        // Lock tree for reading
        tree_->lockRead();

        // Create occupied voxel markers
        visualization_msgs::msg::Marker occupied_marker;
        occupied_marker.header.frame_id = map_frame_;
        occupied_marker.header.stamp = now;
        occupied_marker.ns = "occupied_cells";
        occupied_marker.id = 0;
        occupied_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        occupied_marker.action = visualization_msgs::msg::Marker::ADD;
        occupied_marker.scale.x = tree_->getResolution();
        occupied_marker.scale.y = tree_->getResolution();
        occupied_marker.scale.z = tree_->getResolution();
        occupied_marker.color.r = 1.0;
        occupied_marker.color.g = 0.0;
        occupied_marker.color.b = 0.0;
        occupied_marker.color.a = 0.8;

        // Optional: free voxel markers
        visualization_msgs::msg::Marker free_marker;
        if (publish_free)
        {
            free_marker.header.frame_id = map_frame_;
            free_marker.header.stamp = now;
            free_marker.ns = "free_cells";
            free_marker.id = 1;
            free_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
            free_marker.action = visualization_msgs::msg::Marker::ADD;
            free_marker.scale.x = tree_->getResolution();
            free_marker.scale.y = tree_->getResolution();
            free_marker.scale.z = tree_->getResolution();
            free_marker.color.r = 0.0;
            free_marker.color.g = 1.0;
            free_marker.color.b = 0.0;
            free_marker.color.a = 0.2;
        }

        // Iterate through octree
        for (auto it = tree_->begin_leafs(); it != tree_->end_leafs(); ++it)
        {
            if (tree_->isNodeOccupied(*it))
            {
                geometry_msgs::msg::Point point;
                point.x = it.getX();
                point.y = it.getY();
                point.z = it.getZ();
                occupied_marker.points.push_back(point);
            }
            else if (publish_free)
            {
                geometry_msgs::msg::Point point;
                point.x = it.getX();
                point.y = it.getY();
                point.z = it.getZ();
                free_marker.points.push_back(point);
            }
        }

        tree_->unlockRead();

        // Add markers to array
        if (!occupied_marker.points.empty())
        {
            marker_array.markers.push_back(occupied_marker);
        }

        if (publish_free && !free_marker.points.empty())
        {
            marker_array.markers.push_back(free_marker);
        }

        // Add bounding box visualization if enabled
        if (tree_->bbxSet())
        {
            publishBoundingBox(marker_array, now);
        }

        // Publish
        marker_pub_->publish(marker_array);

        RCLCPP_DEBUG(logger_, "Published occupancy map visualization with %zu occupied voxels",
                     occupied_marker.points.size());
    }

    void OccupancyMapVisualizer::publishBoundingBox(
        visualization_msgs::msg::MarkerArray &marker_array,
        const rclcpp::Time &stamp,
        const std::optional<octomap::point3d> &bbx_min_opt,
        const std::optional<octomap::point3d> &bbx_max_opt)
    {
        // Get bounding box min/max - use provided values or fall back to tree's values
        octomap::point3d bbx_min = bbx_min_opt.has_value() ? bbx_min_opt.value() : tree_->getBBXMin();
        octomap::point3d bbx_max = bbx_max_opt.has_value() ? bbx_max_opt.value() : tree_->getBBXMax();

        // Create LINE_LIST marker for bounding box edges
        visualization_msgs::msg::Marker bbox_marker;
        bbox_marker.header.frame_id = map_frame_;
        bbox_marker.header.stamp = stamp;
        bbox_marker.ns = "bounding_box";
        bbox_marker.id = 100;
        bbox_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        bbox_marker.action = visualization_msgs::msg::Marker::ADD;
        bbox_marker.scale.x = 0.02; // Line width
        bbox_marker.color.r = 0.0;
        bbox_marker.color.g = 0.5;
        bbox_marker.color.b = 1.0;
        bbox_marker.color.a = 1.0;
        bbox_marker.pose.orientation.w = 1.0;

        // Define 8 corners of the bounding box
        std::array<geometry_msgs::msg::Point, 8> corners;

        // Bottom face (z = min)
        corners[0].x = bbx_min.x();
        corners[0].y = bbx_min.y();
        corners[0].z = bbx_min.z();
        corners[1].x = bbx_max.x();
        corners[1].y = bbx_min.y();
        corners[1].z = bbx_min.z();
        corners[2].x = bbx_max.x();
        corners[2].y = bbx_max.y();
        corners[2].z = bbx_min.z();
        corners[3].x = bbx_min.x();
        corners[3].y = bbx_max.y();
        corners[3].z = bbx_min.z();

        // Top face (z = max)
        corners[4].x = bbx_min.x();
        corners[4].y = bbx_min.y();
        corners[4].z = bbx_max.z();
        corners[5].x = bbx_max.x();
        corners[5].y = bbx_min.y();
        corners[5].z = bbx_max.z();
        corners[6].x = bbx_max.x();
        corners[6].y = bbx_max.y();
        corners[6].z = bbx_max.z();
        corners[7].x = bbx_min.x();
        corners[7].y = bbx_max.y();
        corners[7].z = bbx_max.z();

        // Bottom face edges (4 edges)
        bbox_marker.points.push_back(corners[0]);
        bbox_marker.points.push_back(corners[1]);
        bbox_marker.points.push_back(corners[1]);
        bbox_marker.points.push_back(corners[2]);
        bbox_marker.points.push_back(corners[2]);
        bbox_marker.points.push_back(corners[3]);
        bbox_marker.points.push_back(corners[3]);
        bbox_marker.points.push_back(corners[0]);

        // Top face edges (4 edges)
        bbox_marker.points.push_back(corners[4]);
        bbox_marker.points.push_back(corners[5]);
        bbox_marker.points.push_back(corners[5]);
        bbox_marker.points.push_back(corners[6]);
        bbox_marker.points.push_back(corners[6]);
        bbox_marker.points.push_back(corners[7]);
        bbox_marker.points.push_back(corners[7]);
        bbox_marker.points.push_back(corners[4]);

        // Vertical edges connecting top and bottom (4 edges)
        bbox_marker.points.push_back(corners[0]);
        bbox_marker.points.push_back(corners[4]);
        bbox_marker.points.push_back(corners[1]);
        bbox_marker.points.push_back(corners[5]);
        bbox_marker.points.push_back(corners[2]);
        bbox_marker.points.push_back(corners[6]);
        bbox_marker.points.push_back(corners[3]);
        bbox_marker.points.push_back(corners[7]);

        marker_array.markers.push_back(bbox_marker);

        RCLCPP_DEBUG(logger_, "Published bounding box visualization");
    }

} // namespace husky_xarm6_mcr_occupancy_map