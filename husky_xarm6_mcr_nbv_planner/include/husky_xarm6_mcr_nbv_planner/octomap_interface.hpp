#pragma once

#include <rclcpp/rclcpp.hpp>
#include <octomap/OcTree.h>
#include <octomap_msgs/msg/octomap.hpp>
#include "husky_xarm6_mcr_occupancy_map/msg/custom_octomap.hpp"
#include "husky_xarm6_mcr_occupancy_map/semantic_occupancy_map_tree.hpp"
#include "husky_xarm6_mcr_nbv_planner/cluster.hpp"

#include <shared_mutex>
#include <memory>
#include <vector>
#include <variant>
#include <map>

namespace husky_xarm6_mcr_nbv_planner
{
    enum class OctoMapType
    {
        STANDARD,
        SEMANTIC
    };

    class OctoMapInterface
    {
    public:
        OctoMapInterface(const rclcpp::Node::SharedPtr &node,
                         const std::string &octomap_topic = "/nbv/octomap_binary",
                         bool transient_local = true);

        // Common operations (work with both tree types)
        std::shared_ptr<octomap::AbstractOcTree> getTreeSnapshot() const;
        
        std::vector<octomap::point3d> findFrontiers(int min_unknown_neighbors = 1,
                                                    bool use_bbox = false) const;

        std::vector<Cluster> kmeansCluster(const std::vector<octomap::point3d> &points,
                                           int n_clusters = 0,
                                           int max_iters = 50,
                                           double tol = 1e-4) const;

        bool isTreeAvailable() const;
        bool getResolution(double &resolution_out) const;
        bool getBoundingBox(octomap::point3d &min_out, octomap::point3d &max_out) const;
        bool isVoxelOccupied(const octomap::point3d &point) const;
        bool searchOctree(const octomap::point3d &point, octomap::OcTreeNode *&node_out) const;
        int getOctreeDepth() const;
        rclcpp::Time getLastUpdateTime() const;

        // Tree type info
        OctoMapType getTreeType() const;
        bool isSemanticTree() const;
        std::string getTreeTypeString() const;

        // Semantic-specific operations (return false if not semantic tree)
        bool getVoxelSemantic(const octomap::point3d &point, 
                             int32_t &class_id, 
                             float &confidence) const;
        
        std::vector<octomap::point3d> findFrontiersByClass(int32_t class_id,
                                                           float min_confidence = 0.5f,
                                                           int min_unknown_neighbors = 1,
                                                           bool use_bbox = false) const;
        
        // Get counts of occupied voxels per semantic class
        // Returns map of class_id -> count. Returns empty map if not semantic tree.
        std::map<int32_t, size_t> getSemanticClassCounts() const;

        // Type-safe accessors
        std::shared_ptr<octomap::OcTree> getStandardTree() const;
        std::shared_ptr<octomap::SemanticOcTree> getSemanticTree() const;

    private:
        void onOctomap(const husky_xarm6_mcr_occupancy_map::msg::CustomOctomap::SharedPtr msg);

        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<husky_xarm6_mcr_occupancy_map::msg::CustomOctomap>::SharedPtr sub_;

        mutable std::shared_mutex mtx_;
        
        // Store as variant to support both tree types without type erasure overhead
        std::variant<
            std::shared_ptr<octomap::OcTree>,
            std::shared_ptr<octomap::SemanticOcTree>
        > tree_;
        
        OctoMapType tree_type_{OctoMapType::STANDARD};
        double resolution_{0.0};
        octomap::point3d bbox_min_{0.0, 0.0, 0.0};
        octomap::point3d bbox_max_{0.0, 0.0, 0.0};
        bool has_valid_bbox_{false};
        rclcpp::Time last_update_time_;
    };

} // namespace husky_xarm6_mcr_nbv_planner