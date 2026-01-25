/**
 * @file nbv_visualizer.cpp
 * @brief Implementation of RViz visualization for NBV planning
 */

#include "husky_xarm6_mcr_nbv_planner/nbv_visualizer.hpp"
#include <Eigen/Geometry>
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
        const std_msgs::msg::ColorRGBA &color,
        float alpha,
        const std::string &ns)
    {
        auto now = node_->now();
        visualization_msgs::msg::MarkerArray marker_array;

        // Delete previous markers
        visualization_msgs::msg::Marker del_marker;
        del_marker.header.frame_id = map_frame_;
        del_marker.header.stamp = now;
        del_marker.ns = ns;
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
        marker.ns = ns;
        marker.id = 1;
        marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = voxel_size;
        marker.scale.y = voxel_size;
        marker.scale.z = voxel_size;
        marker.color = color;
        marker.color.a = alpha;

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
        bool plot_centers,
        float alpha,
        const std::string &ns)
    {
        auto now = node_->now();
        visualization_msgs::msg::MarkerArray marker_array;

        // Delete previous markers
        visualization_msgs::msg::Marker del_marker;
        del_marker.header.frame_id = map_frame_;
        del_marker.header.stamp = now;
        del_marker.ns = ns;
        del_marker.id = 0;
        del_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(del_marker);

        if (plot_centers)
        {
            visualization_msgs::msg::Marker del_centers;
            del_centers.header.frame_id = map_frame_;
            del_centers.header.stamp = now;
            del_centers.ns = ns + "_centers";
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
            marker.ns = ns;
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = marker.scale.y = marker.scale.z = voxel_size;
            marker.color = colorForLabel(cluster.label);
            marker.color.a = alpha;

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
            centers.ns = ns + "_centers";
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
                                     [](int sum, const Cluster &c)
                                     { return sum + c.size; }));
    }

    visualization_msgs::msg::Marker NBVVisualizer::createAxisTriad(
        const geometry_msgs::msg::Pose &pose,
        int id,
        double axis_length,
        double axis_radius,
        float alpha,
        const std::string &ns)
    {
        // Note: This creates a LINE_LIST marker for compatibility with single-marker return
        // For proper cylinders, use the publishCoordinates function which creates multiple markers
        visualization_msgs::msg::Marker triad;
        triad.header.frame_id = map_frame_;
        triad.header.stamp = node_->now();
        triad.ns = ns;
        triad.id = id;
        triad.type = visualization_msgs::msg::Marker::LINE_LIST;
        triad.action = visualization_msgs::msg::Marker::ADD;
        triad.pose = pose;
        triad.scale.x = axis_radius * 2.0; // line width (thicker for visibility)

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

    std::vector<visualization_msgs::msg::Marker> NBVVisualizer::createAxisTriadCylinders(
        const geometry_msgs::msg::Pose &pose,
        int base_id,
        double axis_length,
        double axis_radius,
        float alpha,
        const std::string &ns)
    {
        std::vector<visualization_msgs::msg::Marker> markers;
        
        // Helper lambda to create a cylinder for one axis
        auto createAxisCylinder = [&](int axis_id, double dx, double dy, double dz, float r, float g, float b) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = map_frame_;
            marker.header.stamp = node_->now();
            marker.ns = ns;
            marker.id = base_id * 3 + axis_id;
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            // Transform the pose to position the cylinder along the axis
            // Cylinder is centered at its pose, oriented along Z axis by default
            // We need to translate it by half the length and rotate it
            
            // Convert pose quaternion to transformation matrix
            Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, 
                                pose.orientation.y, pose.orientation.z);
            Eigen::Vector3d pos(pose.position.x, pose.position.y, pose.position.z);
            
            // Direction vector for this axis
            Eigen::Vector3d axis_dir(dx, dy, dz);
            axis_dir.normalize();
            
            // Position at half the axis length
            Eigen::Vector3d axis_pos = pos + q * (axis_dir * axis_length * 0.5);
            
            // Create rotation to align Z axis with the desired axis direction
            Eigen::Vector3d z_axis(0, 0, 1);
            Eigen::Quaterniond axis_rotation;
            
            // If axis is along +X
            if (std::abs(dx - 1.0) < 0.01) {
                axis_rotation = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0, 1, 0));
            }
            // If axis is along +Y
            else if (std::abs(dy - 1.0) < 0.01) {
                axis_rotation = Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d(1, 0, 0));
            }
            // If axis is along +Z
            else {
                axis_rotation = Eigen::Quaterniond::Identity();
            }
            
            // Combine rotations
            Eigen::Quaterniond final_q = q * axis_rotation;
            
            marker.pose.position.x = axis_pos.x();
            marker.pose.position.y = axis_pos.y();
            marker.pose.position.z = axis_pos.z();
            marker.pose.orientation.x = final_q.x();
            marker.pose.orientation.y = final_q.y();
            marker.pose.orientation.z = final_q.z();
            marker.pose.orientation.w = final_q.w();
            
            marker.scale.x = axis_radius * 2.0; // diameter
            marker.scale.y = axis_radius * 2.0; // diameter  
            marker.scale.z = axis_length;       // height
            
            marker.color.r = r;
            marker.color.g = g;
            marker.color.b = b;
            marker.color.a = alpha;
            
            return marker;
        };
        
        // Create X axis (red)
        markers.push_back(createAxisCylinder(0, 1, 0, 0, 1.0f, 0.0f, 0.0f));
        // Create Y axis (green)
        markers.push_back(createAxisCylinder(1, 0, 1, 0, 0.0f, 1.0f, 0.0f));
        // Create Z axis (blue)
        markers.push_back(createAxisCylinder(2, 0, 0, 1, 0.0f, 0.0f, 1.0f));
        
        return markers;
    }

    void NBVVisualizer::publishCoordinates(
        const std::vector<geometry_msgs::msg::Pose> &poses,
        double axis_length,
        double axis_radius,
        float alpha,
        const std::string &ns)
    {
        auto now = node_->now();
        visualization_msgs::msg::MarkerArray marker_array;

        // // Delete previous markers
        // visualization_msgs::msg::Marker del_marker;
        // del_marker.header.frame_id = map_frame_;
        // del_marker.header.stamp = now;
        // del_marker.ns = ns;
        // del_marker.id = 0;
        // del_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        // marker_array.markers.push_back(del_marker);

        if (poses.empty())
        {
            marker_pub_->publish(marker_array);
            return;
        }

        // Create axis triad cylinders for each pose
        for (size_t i = 0; i < poses.size(); ++i)
        {
            auto cylinders = createAxisTriadCylinders(
                poses[i],
                static_cast<int>(i + 1),
                axis_length,
                axis_radius,
                alpha,
                ns);
            
            // Add all three cylinder markers (X, Y, Z axes)
            for (auto &marker : cylinders)
            {
                marker_array.markers.push_back(marker);
            }
        }

        marker_pub_->publish(marker_array);
        RCLCPP_DEBUG(logger_, "Published %zu candidate views", poses.size());
    }

    void NBVVisualizer::publishCoordinate(
        const geometry_msgs::msg::Pose &pose,
        double axis_length,
        double axis_radius,
        float alpha,
        const std::string &ns)
    {
        // Single pose - wrap in vector and call multi-pose version
        std::vector<geometry_msgs::msg::Pose> poses = {pose};
        publishCoordinates(poses, axis_length, axis_radius, alpha, ns);
    }

    void NBVVisualizer::publishCoordinate(
        const Eigen::Vector3d &position,
        const std::array<double, 4> &orientation,
        double axis_length,
        double axis_radius,
        float alpha,
        const std::string &ns)
    {
        // Convert Eigen types to geometry_msgs::Pose
        geometry_msgs::msg::Pose pose;
        pose.position.x = position.x();
        pose.position.y = position.y();
        pose.position.z = position.z();
        pose.orientation.x = orientation[0];
        pose.orientation.y = orientation[1];
        pose.orientation.z = orientation[2];
        pose.orientation.w = orientation[3];
        
        publishCoordinate(pose, axis_length, axis_radius, alpha, ns);
    }

    void NBVVisualizer::publishViewpoint(
        const Viewpoint &viewpoint,
        double axis_length,
        double axis_radius,
        float alpha,
        const std::string &ns)
    {
        // Convert Viewpoint to geometry_msgs::Pose
        geometry_msgs::msg::Pose pose;
        pose.position.x = viewpoint.position.x();
        pose.position.y = viewpoint.position.y();
        pose.position.z = viewpoint.position.z();
        pose.orientation.x = viewpoint.orientation[0];
        pose.orientation.y = viewpoint.orientation[1];
        pose.orientation.z = viewpoint.orientation[2];
        pose.orientation.w = viewpoint.orientation[3];

        // Publish using existing publishCoordinate function
        publishCoordinate(pose, axis_length, axis_radius, alpha, ns);
    }

    visualization_msgs::msg::Marker NBVVisualizer::createWireframeSphere(
        const octomap::point3d &center,
        double radius,
        double line_width,
        const std_msgs::msg::ColorRGBA &color,
        const std::string &ns)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = map_frame_;
        marker.header.stamp = node_->now();
        marker.ns = ns;
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
        double line_width,
        const std::string &ns)
    {
        auto now = node_->now();
        visualization_msgs::msg::MarkerArray marker_array;

        // Delete previous marker
        visualization_msgs::msg::Marker del_marker;
        del_marker.header.frame_id = map_frame_;
        del_marker.header.stamp = now;
        del_marker.ns = ns;
        del_marker.id = 0;
        del_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(del_marker);

        // Create wireframe sphere
        std_msgs::msg::ColorRGBA color;
        color.r = 1.0f;
        color.g = 0.5f;
        color.b = 0.0f;
        color.a = 0.8f;

        auto sphere = createWireframeSphere(center, radius, line_width, color, ns);
        marker_array.markers.push_back(sphere);

        marker_pub_->publish(marker_array);
        RCLCPP_DEBUG(logger_, "Published target region");
    }

    void NBVVisualizer::publishBoundingBox(
        const octomap::point3d &bbx_min,
        const octomap::point3d &bbx_max,
        const std::string &ns,
        double line_width,
        const std_msgs::msg::ColorRGBA &color)
    {
        auto now = node_->now();
        visualization_msgs::msg::MarkerArray marker_array;

        // Delete previous markers
        visualization_msgs::msg::Marker del_marker;
        del_marker.header.frame_id = map_frame_;
        del_marker.header.stamp = now;
        del_marker.ns = ns;
        del_marker.id = 0;
        del_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(del_marker);

        // Create LINE_LIST marker for bounding box edges
        visualization_msgs::msg::Marker bbox_marker;
        bbox_marker.header.frame_id = map_frame_;
        bbox_marker.header.stamp = now;
        bbox_marker.ns = ns;
        bbox_marker.id = 1;
        bbox_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        bbox_marker.action = visualization_msgs::msg::Marker::ADD;
        bbox_marker.scale.x = line_width; // Line width
        bbox_marker.pose.orientation.w = 1.0;

        // Set color (use provided color or default to cyan)
        if (color.a == 0.0 && color.r == 0.0 && color.g == 0.0 && color.b == 0.0)
        {
            bbox_marker.color.r = 0.0;
            bbox_marker.color.g = 0.5;
            bbox_marker.color.b = 1.0;
            bbox_marker.color.a = 1.0;
        }
        else
        {
            bbox_marker.color = color;
        }

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
        marker_pub_->publish(marker_array);

        RCLCPP_DEBUG(logger_, "Published bounding box visualization");
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
        int id,
        double size,
        const std_msgs::msg::ColorRGBA &color,
        float alpha,
        const std::string &ns)
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
        double size,
        const std_msgs::msg::ColorRGBA &color,
        float alpha,
        const std::string &ns)
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
            marker.color.a = alpha;
        }
        else
        {
            marker.color = color;
            marker.color.a = alpha;
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
