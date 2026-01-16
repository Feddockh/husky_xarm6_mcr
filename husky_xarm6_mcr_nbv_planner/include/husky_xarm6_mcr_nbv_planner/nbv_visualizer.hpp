/**
 * @file nbv_visualizer.hpp
 * @brief RViz visualization for NBV planning components
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <octomap/octomap.h>
#include "husky_xarm6_mcr_nbv_planner/cluster.hpp"

#include <vector>
#include <string>
#include <memory>

namespace husky_xarm6_mcr_nbv_planner
{


    /**
     * @brief Publishes RViz markers for NBV planning visualization
     *
     * Creates marker arrays showing:
     * - Frontier voxels (free voxels adjacent to unknown space)
     * - Cluster assignments and centers
     * - Candidate viewpoint poses
     * - Selected next-best-view
     * - Target region
     */
    class NBVVisualizer
    {
    public:
        /**
         * @brief Constructor
         * @param node ROS2 node
         * @param map_frame Frame ID for markers (default: "world")
         * @param topic Topic name for marker array publishing (default: "nbv_markers")
         */
        NBVVisualizer(
            const rclcpp::Node::SharedPtr &node,
            const std::string &map_frame = "world",
            const std::string &topic = "nbv_markers");

        /**
         * @brief Publish frontier voxels as colored cubes
         * @param frontiers Frontier voxel positions
         * @param voxel_size Size of each voxel cube
         * @param color RGBA color for all frontiers
         */
        void publishFrontiers(
            const std::vector<octomap::point3d> &frontiers,
            double voxel_size,
            const std_msgs::msg::ColorRGBA &color);

        /**
         * @brief Publish clustered frontiers with per-cluster colors
         * @param clusters Vector of clusters, each containing frontier voxel positions
         * @param voxel_size Size of each voxel cube
         * @param plot_centers Whether to plot cluster centers as small 0.1 diameter spheres
         */
        void publishClusteredFrontiers(
            const std::vector<Cluster> &clusters,
            double voxel_size,
            bool plot_centers = false);

        /**
         * @brief Publish candidate viewpoint poses as axes
         * @param poses Candidate camera poses
         * @param axis_length Length of coordinate axes
         * @param axis_radius Thickness of axes
         */
        void publishCandidateViews(
            const std::vector<geometry_msgs::msg::Pose> &poses,
            double axis_length = 0.20,
            double axis_radius = 0.01);

        /**
         * @brief Publish selected next-best-view as highlighted axis
         * @param pose Selected NBV camera pose
         * @param axis_length Length of coordinate axes
         * @param axis_radius Thickness of axes
         */
        void publishBestView(
            const geometry_msgs::msg::Pose &pose,
            double axis_length = 0.30,
            double axis_radius = 0.02);

        /**
         * @brief Publish target region as wireframe sphere
         * @param center Center of target region
         * @param radius Radius of target region
         * @param line_width Width of wireframe lines
         */
        void publishTargetRegion(
            const octomap::point3d &center,
            double radius,
            double line_width = 0.02);

        /**
         * @brief Clear all visualizations in a namespace
         * @param ns_suffix Namespace suffix (empty = all namespaces)
         */
        void clearAll(const std::string &ns_suffix = "");

        /**
         * @brief Clear all marker namespaces at once
         */
        void clearAllMarkers();

        /**
         * @brief Publish a single point marker
         * @param point 3D point position
         * @param ns Namespace for the marker
         * @param id Unique ID for the marker
         * @param size Size of the point marker (default: 0.02m)
         * @param color Optional color for the point (default: green)
         */
        void publishPoint(
            const octomap::point3d &point,
            const std::string &ns,
            int id,
            double size = 0.02,
            const std_msgs::msg::ColorRGBA &color = std_msgs::msg::ColorRGBA());

        /**
         * @brief Publish multiple points as a single marker (more efficient)
         * @param points Vector of 3D point positions
         * @param ns Namespace for the marker
         * @param size Size of each point (default: 0.02m)
         * @param color Optional color for all points (default: green)
         */
        void publishPoints(
            const std::vector<octomap::point3d> &points,
            const std::string &ns,
            double size = 0.02,
            const std_msgs::msg::ColorRGBA &color = std_msgs::msg::ColorRGBA());

        /**
         * @brief Clear specific visualization namespace
         */
        void clearFrontiers();
        void clearClusteredFrontiers();
        void clearClusterCenters();
        void clearCandidateViews();
        void clearBestView();
        void clearTargetRegion();

    private:
        /**
         * @brief Generate deterministic color from cluster label
         */
        std_msgs::msg::ColorRGBA colorForLabel(int label, float alpha = 0.85f);

        /**
         * @brief Create wireframe sphere marker
         */
        visualization_msgs::msg::Marker createWireframeSphere(
            const octomap::point3d &center,
            double radius,
            double line_width,
            const std_msgs::msg::ColorRGBA &color);

        /**
         * @brief Create coordinate axis triad marker
         */
        visualization_msgs::msg::Marker createAxisTriad(
            const geometry_msgs::msg::Pose &pose,
            int id,
            const std::string &ns,
            double axis_length,
            double axis_radius,
            float alpha = 1.0f);

        /**
         * @brief Helper to create geometry_msgs::Point
         */
        geometry_msgs::msg::Point toPoint(const octomap::point3d &p);

        rclcpp::Node::SharedPtr node_;
        std::string map_frame_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
        rclcpp::Logger logger_;
    };

    using NBVVisualizerPtr = std::shared_ptr<NBVVisualizer>;

} // namespace husky_xarm6_mcr_nbv_planner