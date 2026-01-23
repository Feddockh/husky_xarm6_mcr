#pragma once

#include <rclcpp/rclcpp.hpp>
#include <octomap/OcTree.h>
#include <octomap_msgs/msg/octomap.hpp>
#include "husky_xarm6_mcr_nbv_planner/cluster.hpp"

#include <shared_mutex>
#include <memory>
#include <vector>
#include <random>

namespace husky_xarm6_mcr_nbv_planner
{

    class OctoMapInterface
    {
    public:
        OctoMapInterface(const rclcpp::Node::SharedPtr &node,
                         const std::string &octomap_topic = "/nbv/octomap_binary",
                         bool transient_local = true);

        // Returns a snapshot pointer you can query without holding locks.
        std::shared_ptr<octomap::OcTree> getTreeSnapshot() const;

        // Frontier extraction: free leaf voxels with >= min_unknown_neighbors in 6-neighborhood.
        std::vector<octomap::point3d> findFrontiers(int min_unknown_neighbors = 1,
                                                    bool use_bbox = false) const;

        // K-means clustering (Lloyd). If n_clusters <= 0, uses heuristic like your python code.
        std::vector<Cluster> kmeansCluster(const std::vector<octomap::point3d> &points,
                                           int n_clusters = 0,
                                           int max_iters = 50,
                                           double tol = 1e-4) const;

        bool isTreeAvailable() const;

        bool getResolution(double &resolution_out) const;

        bool getBoundingBox(octomap::point3d &min_out,
                            octomap::point3d &max_out) const;

        bool isVoxelOccupied(const octomap::point3d &point) const;

        bool searchOctree(const octomap::point3d &point,
                            octomap::OcTreeNode *&node_out) const;

        int getOctreeDepth() const;

    private:
        void onOctomap(const octomap_msgs::msg::Octomap::SharedPtr msg);

        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr sub_;

        mutable std::shared_mutex mtx_;
        std::shared_ptr<octomap::OcTree> tree_;
        double resolution_{0.0};
        octomap::point3d bbox_min_{0.0, 0.0, 0.0};
        octomap::point3d bbox_max_{0.0, 0.0, 0.0};
        bool has_valid_bbox_{false};
    };

} // namespace husky_xarm6_mcr_nbv_planner