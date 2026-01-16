/**
 * @file nbv_visualizer.cpp
 * @brief Implementation of RViz visualization for NBV planning
 */

#include "husky_xarm6_mcr_nbv_planner/nbv_visualizer.hpp"
#include <unordered_map>
#include <numeric>
#include <cmath>

namespace husky_xarm6_mcr_nbv_planner
{

    NBVVisualizer::NBVVisualizer(
        const rclcpp::Node::SharedPtr &node,
        const std::string &map_frame,
        const std::string &topic)
        : node_(node),
          map_frame_(map_frame),
          logger_(node_->get_logger())
    {
        // Create marker publisher with default QoS (VOLATILE) to match RViz expectations
        marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            topic, rclcpp::QoS(10));

        RCLCPP_INFO(logger_, "NBVVisualizer created for frame '%s', publishing to '%s'", 
                    map_frame_.c_str(), topic.c_str());
    }

    std_msgs::msg::ColorRGBA NBVVisualizer::colorForLabel(int label, float alpha)
    {
        std_msgs::msg::ColorRGBA color;

        if (label < 0)
        {
            // Gray for noise
            color.r = 0.6f;
            color.g = 0.6f;
            color.b = 0.6f;
            color.a = alpha;
            return color;
        }

        // Golden ratio for good color spread
        const double h = std::fmod(0.618033988749895 * (label + 1), 1.0);
        const double s = 0.85;
        const double v = 0.95;

        // HSV to RGB conversion
        const double i = std::floor(h * 6.0);
        const double f = h * 6.0 - i;
        const double p = v * (1.0 - s);
        const double q = v * (1.0 - f * s);
        const double t = v * (1.0 - (1.0 - f) * s);

        double r = 0, g = 0, b = 0;
        switch (static_cast<int>(i) % 6)
        {
        case 0:
            r = v;
            g = t;
            b = p;
            break;
        case 1:
            r = q;
            g = v;
            b = p;
            break;
        case 2:
            r = p;
            g = v;
            b = t;
            break;
        case 3:
            r = p;
            g = q;
            b = v;
            break;
        case 4:
            r = t;
            g = p;
            b = v;
            break;
        case 5:
            r = v;
            g = p;
            b = q;
            break;
        }

        color.r = static_cast<float>(r);
        color.g = static_cast<float>(g);
        color.b = static_cast<float>(b);
        color.a = alpha;

        return color;
    }

    geometry_msgs::msg::Point NBVVisualizer::toPoint(const octomap::point3d &p)
    {
        geometry_msgs::msg::Point point;
        point.x = p.x();
        point.y = p.y();
        point.z = p.z();
        return point;
    }

    void NBVVisualizer::publishFrontiers(
        const std::vector<octomap::point3d> &frontiers,
        double voxel_size,
        const std_msgs::msg::ColorRGBA &color)
    {
        auto now = node_->now();
        visualization_msgs::msg::MarkerArray marker_array;

        // Delete previous markers
        visualization_msgs::msg::Marker del_marker;
        del_marker.header.frame_id = map_frame_;
        del_marker.header.stamp = now;
        del_marker.ns = "frontiers";
        del_marker.id = 0;
        del_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(del_marker);

        if (frontiers.empty())
        {
            marker_pub_->publish(marker_array);
            return;
        }

        // Create CUBE_LIST marker for all frontiers
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = map_frame_;
        marker.header.stamp = now;
        marker.ns = "frontiers";
        marker.id = 1;
        marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = voxel_size;
        marker.scale.y = voxel_size;
        marker.scale.z = voxel_size;
        marker.color = color;

        for (const auto &frontier : frontiers)
        {
            marker.points.push_back(toPoint(frontier));
        }

        marker_array.markers.push_back(marker);
        marker_pub_->publish(marker_array);

        RCLCPP_DEBUG(logger_, "Published %zu frontier voxels", frontiers.size());
    }

    void NBVVisualizer::publishClusteredFrontiers(
        const std::vector<Cluster> &clusters,
        double voxel_size,
        bool plot_centers)
    {
        auto now = node_->now();
        visualization_msgs::msg::MarkerArray marker_array;

        // Delete previous markers
        visualization_msgs::msg::Marker del_marker;
        del_marker.header.frame_id = map_frame_;
        del_marker.header.stamp = now;
        del_marker.ns = "clustered_frontiers";
        del_marker.id = 0;
        del_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(del_marker);

        if (plot_centers)
        {
            visualization_msgs::msg::Marker del_centers;
            del_centers.header.frame_id = map_frame_;
            del_centers.header.stamp = now;
            del_centers.ns = "cluster_centers";
            del_centers.id = 0;
            del_centers.action = visualization_msgs::msg::Marker::DELETEALL;
            marker_array.markers.push_back(del_centers);
        }

        if (clusters.empty())
        {
            marker_pub_->publish(marker_array);
            return;
        }

        // Create one CUBE_LIST marker per cluster
        int id = 1;
        for (const auto &cluster : clusters)
        {
            if (cluster.points.empty())
                continue;

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = map_frame_;
            marker.header.stamp = now;
            marker.ns = "clustered_frontiers";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = marker.scale.y = marker.scale.z = voxel_size;
            marker.color = colorForLabel(cluster.label);

            for (const auto &point : cluster.points)
            {
                marker.points.push_back(toPoint(point));
            }

            marker_array.markers.push_back(marker);
        }

        // Add cluster centers as small spheres
        if (plot_centers)
        {
            visualization_msgs::msg::Marker centers;
            centers.header.frame_id = map_frame_;
            centers.header.stamp = now;
            centers.ns = "cluster_centers";
            centers.id = 1;
            centers.type = visualization_msgs::msg::Marker::SPHERE_LIST;
            centers.action = visualization_msgs::msg::Marker::ADD;
            centers.pose.orientation.w = 1.0;
            centers.scale.x = centers.scale.y = centers.scale.z = 0.1;
            centers.color.r = 1.0f;
            centers.color.g = 1.0f;
            centers.color.b = 1.0f;
            centers.color.a = 1.0f;

            for (const auto &cluster : clusters)
            {
                if (!cluster.points.empty())
                {
                    centers.points.push_back(toPoint(cluster.center));
                }
            }

            marker_array.markers.push_back(centers);
        }

        marker_pub_->publish(marker_array);
        RCLCPP_DEBUG(logger_, "Published %zu clusters with %d total frontier voxels",
                     clusters.size(), 
                     std::accumulate(clusters.begin(), clusters.end(), 0,
                                     [](int sum, const Cluster &c) { return sum + c.size; }));
    }

    visualization_msgs::msg::Marker NBVVisualizer::createAxisTriad(
        const geometry_msgs::msg::Pose &pose,
        int id,
        const std::string &ns,
        double axis_length,
        double axis_radius,
        float alpha)
    {
        visualization_msgs::msg::Marker triad;
        triad.header.frame_id = map_frame_;
        triad.header.stamp = node_->now();
        triad.ns = ns;
        triad.id = id;
        triad.type = visualization_msgs::msg::Marker::LINE_LIST;
        triad.action = visualization_msgs::msg::Marker::ADD;
        triad.pose = pose;
        triad.scale.x = axis_radius; // line width

        // Define axis lines in local frame
        geometry_msgs::msg::Point origin, x_end, y_end, z_end;
        origin.x = origin.y = origin.z = 0.0;
        x_end.x = axis_length;
        x_end.y = x_end.z = 0.0;
        y_end.y = axis_length;
        y_end.x = y_end.z = 0.0;
        z_end.z = axis_length;
        z_end.x = z_end.y = 0.0;

        // X axis (red)
        triad.points.push_back(origin);
        triad.points.push_back(x_end);
        // Y axis (green)
        triad.points.push_back(origin);
        triad.points.push_back(y_end);
        // Z axis (blue)
        triad.points.push_back(origin);
        triad.points.push_back(z_end);

        // Colors per vertex
        triad.colors.resize(6);
        // X axis red
        triad.colors[0].r = 1.0f;
        triad.colors[0].a = alpha;
        triad.colors[1].r = 1.0f;
        triad.colors[1].a = alpha;
        // Y axis green
        triad.colors[2].g = 1.0f;
        triad.colors[2].a = alpha;
        triad.colors[3].g = 1.0f;
        triad.colors[3].a = alpha;
        // Z axis blue
        triad.colors[4].b = 1.0f;
        triad.colors[4].a = alpha;
        triad.colors[5].b = 1.0f;
        triad.colors[5].a = alpha;

        return triad;
    }

    void NBVVisualizer::publishCandidateViews(
        const std::vector<geometry_msgs::msg::Pose> &poses,
        double axis_length,
        double axis_radius)
    {
        auto now = node_->now();
        visualization_msgs::msg::MarkerArray marker_array;

        // Delete previous markers
        visualization_msgs::msg::Marker del_marker;
        del_marker.header.frame_id = map_frame_;
        del_marker.header.stamp = now;
        del_marker.ns = "candidate_views";
        del_marker.id = 0;
        del_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(del_marker);

        if (poses.empty())
        {
            marker_pub_->publish(marker_array);
            return;
        }

        // Create axis triad for each pose
        for (size_t i = 0; i < poses.size(); ++i)
        {
            auto triad = createAxisTriad(
                poses[i],
                static_cast<int>(i + 1),
                "candidate_views",
                axis_length,
                axis_radius,
                0.5f); // semi-transparent
            marker_array.markers.push_back(triad);
        }

        marker_pub_->publish(marker_array);
        RCLCPP_DEBUG(logger_, "Published %zu candidate views", poses.size());
    }

    void NBVVisualizer::publishBestView(
        const geometry_msgs::msg::Pose &pose,
        double axis_length,
        double axis_radius)
    {
        auto now = node_->now();
        visualization_msgs::msg::MarkerArray marker_array;

        // Delete previous marker
        visualization_msgs::msg::Marker del_marker;
        del_marker.header.frame_id = map_frame_;
        del_marker.header.stamp = now;
        del_marker.ns = "best_view";
        del_marker.id = 0;
        del_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(del_marker);

        // Create highlighted axis triad
        auto triad = createAxisTriad(
            pose,
            1,
            "best_view",
            axis_length,
            axis_radius,
            1.0f); // fully opaque

        marker_array.markers.push_back(triad);
        marker_pub_->publish(marker_array);

        RCLCPP_DEBUG(logger_, "Published best view");
    }

    visualization_msgs::msg::Marker NBVVisualizer::createWireframeSphere(
        const octomap::point3d &center,
        double radius,
        double line_width,
        const std_msgs::msg::ColorRGBA &color)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = map_frame_;
        marker.header.stamp = node_->now();
        marker.ns = "target_region";
        marker.id = 1;
        marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position = toPoint(center);
        marker.pose.orientation.w = 1.0;
        marker.scale.x = line_width;
        marker.color = color;

        // Create wireframe sphere using latitude/longitude lines
        const int lat_segments = 12;
        const int lon_segments = 16;

        // Latitude circles
        for (int i = 0; i <= lat_segments; ++i)
        {
            double lat = M_PI * (double(i) / lat_segments - 0.5);
            double y = radius * std::sin(lat);
            double r = radius * std::cos(lat);

            for (int j = 0; j < lon_segments; ++j)
            {
                double lon1 = 2.0 * M_PI * double(j) / lon_segments;
                double lon2 = 2.0 * M_PI * double(j + 1) / lon_segments;

                geometry_msgs::msg::Point p1, p2;
                p1.x = r * std::cos(lon1);
                p1.y = y;
                p1.z = r * std::sin(lon1);

                p2.x = r * std::cos(lon2);
                p2.y = y;
                p2.z = r * std::sin(lon2);

                marker.points.push_back(p1);
                marker.points.push_back(p2);
            }
        }

        // Longitude lines
        for (int j = 0; j < lon_segments; ++j)
        {
            double lon = 2.0 * M_PI * double(j) / lon_segments;

            for (int i = 0; i < lat_segments; ++i)
            {
                double lat1 = M_PI * (double(i) / lat_segments - 0.5);
                double lat2 = M_PI * (double(i + 1) / lat_segments - 0.5);

                geometry_msgs::msg::Point p1, p2;
                p1.x = radius * std::cos(lat1) * std::cos(lon);
                p1.y = radius * std::sin(lat1);
                p1.z = radius * std::cos(lat1) * std::sin(lon);

                p2.x = radius * std::cos(lat2) * std::cos(lon);
                p2.y = radius * std::sin(lat2);
                p2.z = radius * std::cos(lat2) * std::sin(lon);

                marker.points.push_back(p1);
                marker.points.push_back(p2);
            }
        }

        return marker;
    }

    void NBVVisualizer::publishTargetRegion(
        const octomap::point3d &center,
        double radius,
        double line_width)
    {
        auto now = node_->now();
        visualization_msgs::msg::MarkerArray marker_array;

        // Delete previous marker
        visualization_msgs::msg::Marker del_marker;
        del_marker.header.frame_id = map_frame_;
        del_marker.header.stamp = now;
        del_marker.ns = "target_region";
        del_marker.id = 0;
        del_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(del_marker);

        // Create wireframe sphere
        std_msgs::msg::ColorRGBA color;
        color.r = 1.0f;
        color.g = 0.5f;
        color.b = 0.0f;
        color.a = 0.8f;

        auto sphere = createWireframeSphere(center, radius, line_width, color);
        marker_array.markers.push_back(sphere);

        marker_pub_->publish(marker_array);
        RCLCPP_DEBUG(logger_, "Published target region");
    }

    void NBVVisualizer::clearAll(const std::string &ns_suffix)
    {
        auto now = node_->now();
        visualization_msgs::msg::MarkerArray marker_array;

        visualization_msgs::msg::Marker del_marker;
        del_marker.header.frame_id = map_frame_;
        del_marker.header.stamp = now;
        del_marker.ns = ns_suffix.empty() ? "" : ns_suffix;
        del_marker.id = 0;
        del_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(del_marker);

        marker_pub_->publish(marker_array);
        RCLCPP_DEBUG(logger_, "Cleared all markers in namespace '%s'", ns_suffix.c_str());
    }

    void NBVVisualizer::clearFrontiers()
    {
        auto now = node_->now();
        visualization_msgs::msg::MarkerArray marker_array;

        visualization_msgs::msg::Marker del_marker;
        del_marker.header.frame_id = map_frame_;
        del_marker.header.stamp = now;
        del_marker.ns = "frontiers";
        del_marker.id = 0;
        del_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(del_marker);

        marker_pub_->publish(marker_array);
    }

    void NBVVisualizer::clearClusteredFrontiers()
    {
        auto now = node_->now();
        visualization_msgs::msg::MarkerArray marker_array;

        visualization_msgs::msg::Marker del_marker;
        del_marker.header.frame_id = map_frame_;
        del_marker.header.stamp = now;
        del_marker.ns = "clustered_frontiers";
        del_marker.id = 0;
        del_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(del_marker);

        marker_pub_->publish(marker_array);
    }

    void NBVVisualizer::clearClusterCenters()
    {
        auto now = node_->now();
        visualization_msgs::msg::MarkerArray marker_array;

        visualization_msgs::msg::Marker del_marker;
        del_marker.header.frame_id = map_frame_;
        del_marker.header.stamp = now;
        del_marker.ns = "cluster_centers";
        del_marker.id = 0;
        del_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(del_marker);

        del_marker.ns = "cluster_centers_labels";
        marker_array.markers.push_back(del_marker);

        marker_pub_->publish(marker_array);
    }

    void NBVVisualizer::clearCandidateViews()
    {
        auto now = node_->now();
        visualization_msgs::msg::MarkerArray marker_array;

        visualization_msgs::msg::Marker del_marker;
        del_marker.header.frame_id = map_frame_;
        del_marker.header.stamp = now;
        del_marker.ns = "candidate_views";
        del_marker.id = 0;
        del_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(del_marker);

        marker_pub_->publish(marker_array);
    }

    void NBVVisualizer::clearBestView()
    {
        auto now = node_->now();
        visualization_msgs::msg::MarkerArray marker_array;

        visualization_msgs::msg::Marker del_marker;
        del_marker.header.frame_id = map_frame_;
        del_marker.header.stamp = now;
        del_marker.ns = "best_view";
        del_marker.id = 0;
        del_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(del_marker);

        marker_pub_->publish(marker_array);
    }

    void NBVVisualizer::clearTargetRegion()
    {
        auto now = node_->now();
        visualization_msgs::msg::MarkerArray marker_array;

        visualization_msgs::msg::Marker del_marker;
        del_marker.header.frame_id = map_frame_;
        del_marker.header.stamp = now;
        del_marker.ns = "target_region";
        del_marker.id = 0;
        del_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(del_marker);

        marker_pub_->publish(marker_array);
    }

    void NBVVisualizer::clearAllMarkers()
    {
        // Clear all known namespaces
        clearFrontiers();
        clearClusteredFrontiers();
        clearClusterCenters();
        clearCandidateViews();
        clearBestView();
        clearTargetRegion();
    }

    void NBVVisualizer::publishPoint(
        const octomap::point3d &point,
        const std::string &ns,
        int id,
        double size,
        const std_msgs::msg::ColorRGBA &color)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = map_frame_;
        marker.header.stamp = node_->now();
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.pose.position = toPoint(point);
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = size;
        marker.scale.y = size;
        marker.scale.z = size;
        
        // Use provided color or default to green
        if (color.a == 0.0 && color.r == 0.0 && color.g == 0.0 && color.b == 0.0)
        {
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.8f;
        }
        else
        {
            marker.color = color;
        }
        
        // Publish single marker
        visualization_msgs::msg::MarkerArray marker_array;
        marker_array.markers.push_back(marker);
        marker_pub_->publish(marker_array);
    }

    void NBVVisualizer::publishPoints(
        const std::vector<octomap::point3d> &points,
        const std::string &ns,
        double size,
        const std_msgs::msg::ColorRGBA &color)
    {
        if (points.empty())
        {
            return;
        }

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = map_frame_;
        marker.header.stamp = node_->now();
        marker.ns = ns;
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::POINTS;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        
        // Set point size
        marker.scale.x = size;
        marker.scale.y = size;
        
        // Use provided color or default to green
        if (color.a == 0.0 && color.r == 0.0 && color.g == 0.0 && color.b == 0.0)
        {
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.8f;
        }
        else
        {
            marker.color = color;
        }
        
        // Add all points
        for (const auto &point : points)
        {
            marker.points.push_back(toPoint(point));
        }
        
        // Publish marker array
        visualization_msgs::msg::MarkerArray marker_array;
        marker_array.markers.push_back(marker);
        marker_pub_->publish(marker_array);
        
        RCLCPP_DEBUG(logger_, "Published %zu points in namespace '%s'", points.size(), ns.c_str());
    }

} // namespace husky_xarm6_mcr_nbv_planner
