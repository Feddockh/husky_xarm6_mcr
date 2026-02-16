#pragma once

#include <rclcpp/rclcpp.hpp>
#include <octomap/OcTree.h>
#include <octomap_msgs/msg/octomap.hpp>
#include "husky_xarm6_mcr_occupancy_map/msg/custom_octomap.hpp"
#include "husky_xarm6_mcr_occupancy_map/semantic_occupancy_map_tree.hpp"
#include "husky_xarm6_mcr_nbv_planner/nbv_types.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

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

        // Points returned in specified frame (default: octomap frame)
        std::vector<octomap::point3d> findFrontiers(int min_unknown_neighbors = 1,
                                                    bool use_bbox = false,
                                                    const std::string &frame_id = "") const;

        // Points accepted and returned in specified frame (default: octomap frame)
        std::vector<Cluster> kmeansCluster(const std::vector<octomap::point3d> &points,
                                           int n_clusters = 0,
                                           int max_iters = 50,
                                           double tol = 1e-4,
                                           const std::string &frame_id = "") const;

        bool isTreeAvailable() const {
            std::shared_lock lk(mtx_);
            return std::visit([](auto&& tree) { return tree != nullptr; }, tree_);
        }
        double getResolution() const {
            std::shared_lock lk(mtx_);
            return resolution_;
        }
        // Bounding box returned in specified frame (default: octomap frame)
        bool getBoundingBox(octomap::point3d &min_out, octomap::point3d &max_out,
                           const std::string &frame_id = "") const {
            if (!has_valid_bbox_) {
                return false;
            }
            octomap::point3d min_transformed = bbox_min_;
            octomap::point3d max_transformed = bbox_max_;
            
            // Transform if different frame requested
            std::string target_frame = frame_id.empty() ? octomap_frame_id_ : frame_id;
            if (target_frame != octomap_frame_id_) {
                if (!transformPoint(bbox_min_, octomap_frame_id_, target_frame, min_transformed) ||
                    !transformPoint(bbox_max_, octomap_frame_id_, target_frame, max_transformed)) {
                    return false;
                }
            }
            
            min_out = min_transformed;
            max_out = max_transformed;
            return true;
        }
        // Point accepted in specified frame (default: octomap frame)
        bool isVoxelOccupied(const octomap::point3d &point,
                            const std::string &frame_id = "") const;
        // Point accepted in specified frame (default: octomap frame)
        bool searchOctree(const octomap::point3d &point, octomap::OcTreeNode *&node_out,
                         const std::string &frame_id = "") const;
        int getOctreeDepth() const;
        rclcpp::Time getLastUpdateTime() const {
            std::shared_lock lk(mtx_);
            return last_update_time_;
        }

        // Tree type info
        OctoMapType getTreeType() const {
            std::shared_lock lk(mtx_);
            return tree_type_;
        }
        bool isSemanticTree() const {
            return getTreeType() == OctoMapType::SEMANTIC;
        }
        std::string getTreeTypeString() const {
            std::shared_lock lk(mtx_);
            return tree_type_ == OctoMapType::SEMANTIC ? "SemanticOcTree" : "OcTree";
        }

        // Semantic-specific operations (return false if not semantic tree)
        // Point accepted in specified frame (default: octomap frame)
        bool getVoxelSemantic(const octomap::point3d &point,
                              int32_t &class_id,
                              float &confidence,
                              const std::string &frame_id = "") const;

        // Points returned in specified frame (default: octomap frame)
        std::vector<octomap::point3d> findFrontiersByClass(int32_t class_id,
                                                           float min_confidence = 0.5f,
                                                           int min_unknown_neighbors = 1,
                                                           bool use_bbox = false,
                                                           const std::string &frame_id = "") const;

        // Get counts of occupied voxels per semantic class
        // Returns map of class_id -> count. Returns empty map if not semantic tree.
        std::map<int32_t, size_t> getSemanticClassCounts() const;

        // Type-safe accessors
        std::shared_ptr<octomap::OcTree> getStandardTree() const {
            std::shared_lock lk(mtx_);
            if (tree_type_ == OctoMapType::STANDARD) {
                return std::get<std::shared_ptr<octomap::OcTree>>(tree_);
            }
            return nullptr;
        }
        std::shared_ptr<octomap::SemanticOcTree> getSemanticTree() const {
            std::shared_lock lk(mtx_);
            if (tree_type_ == OctoMapType::SEMANTIC) {
                return std::get<std::shared_ptr<octomap::SemanticOcTree>>(tree_);
            }
            return nullptr;
        }

        // Ground truth evaluation
        /**
         * @brief Load ground truth semantic points from YAML file
         * @param yaml_file_path Path to the YAML file containing marker positions
         * @param frame_id Frame in which ground truth points are specified (if empty, uses frame from YAML or defaults to octomap frame)
         * @return true if loaded successfully, false otherwise
         */
        bool loadGroundTruthSemantics(const std::string &yaml_file_path,
                                     const std::string &frame_id = "");

        /**
         * @brief Match semantic clusters to ground truth points
         * @param clusters Vector of semantic clusters to match
         * @param threshold_radius Maximum distance between cluster centroid and ground truth point for a match
         * @param verbose If true, print detailed matching information
         * @param frame_id Frame in which clusters are specified (default: octomap frame)
         * @return MatchResult containing correct matches, class mismatches, and unmatched points/clusters
         */
        MatchResult matchClustersToGroundTruth(const std::vector<Cluster> &clusters,
                                               double threshold_radius = 0.2,
                                               bool verbose = false,
                                               const std::string &frame_id = "") const;

        /**
         * @brief Compute and print evaluation metrics from match results
         * @param result MatchResult from matchClustersToGroundTruth
         * @param verbose If true, print detailed lists of unmatched points and clusters
         */
        std::vector<ClassMetrics> evaluateMatchResults(const MatchResult &result, bool verbose = false) const;

        /**
         * @brief Evaluate semantic octomap against ground truth
         * @param threshold_radius Maximum distance between cluster centroid and ground truth point for a match
         * @param verbose If true, print detailed evaluation information
         * @param frame_id Frame in which to perform evaluation (default: octomap frame)
         */
        std::vector<ClassMetrics> evaluateSemanticOctomap(double threshold_radius = 0.2, 
                                                          bool verbose = false,
                                                          const std::string &frame_id = "");

        /**
         * @brief Get the loaded ground truth points
         * @param frame_id Frame in which to return points (default: octomap frame)
         * @return Vector of ground truth semantic points
         */
        std::vector<SemanticPoint> getGroundTruthPoints(const std::string &frame_id = "") const;

        /**
         * @brief Get the frame ID of the loaded ground truth points
         * @return Frame ID string (empty if no GT loaded)
         */
        std::string getGroundTruthFrameId() const { return gt_frame_id_; }

        /**
         * @brief Get the frame ID of the current octomap
         * @return Frame ID string (empty if no octomap loaded)
         */
        std::string getOctomapFrameId() const { 
            std::shared_lock lk(mtx_);
            return octomap_frame_id_;
        }

        /**
         * @brief Cluster semantic voxels by class using connected component analysis
         * @param verbose If true, print cluster statistics
         * @param frame_id Frame in which to return clusters (default: octomap frame)
         * @return Vector of semantic clusters (excludes background class)
         */
        std::vector<Cluster> clusterSemanticVoxels(bool verbose = false,
                                                   const std::string &frame_id = "") const;

        /**
         * @brief Print statistics about the current octomap
         * Logs resolution, node counts, occupied/free voxels, and semantic class info (if semantic tree)
         */
        void printOctomapStats() const;

    private:
        void onOctomap(const husky_xarm6_mcr_occupancy_map::msg::CustomOctomap::SharedPtr msg);
        
        /**
         * @brief Transform a point between coordinate frames using TF2
         * @param point_in Input point
         * @param source_frame Frame the point is currently in
         * @param target_frame Frame to transform the point to
         * @param point_out Output transformed point
         * @return true if transformation successful, false otherwise
         */
        bool transformPoint(const octomap::point3d &point_in,
                           const std::string &source_frame,
                           const std::string &target_frame,
                           octomap::point3d &point_out) const;

        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<husky_xarm6_mcr_occupancy_map::msg::CustomOctomap>::SharedPtr sub_;

        mutable std::shared_mutex mtx_;

        // Store as variant to support both tree types without type erasure overhead
        std::variant<
            std::shared_ptr<octomap::OcTree>,
            std::shared_ptr<octomap::SemanticOcTree>>
            tree_;

        OctoMapType tree_type_{OctoMapType::STANDARD};
        double resolution_{0.0};
        octomap::point3d bbox_min_{0.0, 0.0, 0.0};
        octomap::point3d bbox_max_{0.0, 0.0, 0.0};
        bool has_valid_bbox_{false};
        rclcpp::Time last_update_time_;

        // Ground truth data
        std::vector<SemanticPoint> gt_points_;
        std::vector<int> gt_classes_;
        std::string gt_frame_id_;  // Frame ID for ground truth points
        
        // Octomap frame
        std::string octomap_frame_id_;  // Frame ID for octomap
        
        // TF2 for coordinate transformations
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    };

} // namespace husky_xarm6_mcr_nbv_planner