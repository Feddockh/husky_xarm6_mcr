#include "husky_xarm6_mcr_nbv_planner/octomap_interface.hpp"
#include "husky_xarm6_mcr_occupancy_map/semantic_occupancy_map_tree.hpp"

#include <octomap_msgs/conversions.h>
#include <cmath>
#include <limits>
#include <random>
#include <numeric>

namespace husky_xarm6_mcr_nbv_planner
{

    OctoMapInterface::OctoMapInterface(const rclcpp::Node::SharedPtr &node,
                                       const std::string &octomap_topic,
                                       bool transient_local) : node_(node)
    {
        rclcpp::QoS qos(1);
        if (transient_local)
            qos.transient_local();

        sub_ = node_->create_subscription<husky_xarm6_mcr_occupancy_map::msg::CustomOctomap>(
            octomap_topic, qos,
            std::bind(&OctoMapInterface::onOctomap, this, std::placeholders::_1));

        RCLCPP_INFO(node_->get_logger(), "OctoMapInterface subscribing to %s", octomap_topic.c_str());
    }

    void OctoMapInterface::onOctomap(const husky_xarm6_mcr_occupancy_map::msg::CustomOctomap::SharedPtr msg)
    {
        // Use custom conversion that handles semantic trees
        std::unique_ptr<octomap::AbstractOcTree> abs(husky_xarm6_mcr_occupancy_map::semanticMsgToMap(msg->octomap));
        if (!abs)
        {
            RCLCPP_WARN(node_->get_logger(), "Failed to deserialize octomap");
            return;
        }

        // Determine tree type from the message ID
        OctoMapType new_type = OctoMapType::STANDARD;
        if (msg->octomap.id == "SemanticOcTree")
        {
            new_type = OctoMapType::SEMANTIC;
        }

        std::unique_lock lk(mtx_);

        // Try to cast to appropriate type
        if (new_type == OctoMapType::SEMANTIC)
        {
            auto *sem_tree = dynamic_cast<octomap::SemanticOcTree*>(abs.get());
            if (sem_tree)
            {
                tree_ = std::shared_ptr<octomap::SemanticOcTree>(sem_tree);
                abs.release(); // Don't double-delete
                tree_type_ = OctoMapType::SEMANTIC;
                RCLCPP_INFO(node_->get_logger(), "Loaded semantic octomap");
            }
            else
            {
                RCLCPP_WARN(node_->get_logger(), "Expected semantic tree but cast failed");
                return;
            }
        }
        else
        {
            auto *std_tree = dynamic_cast<octomap::OcTree*>(abs.get());
            if (std_tree)
            {
                tree_ = std::shared_ptr<octomap::OcTree>(std_tree);
                abs.release();
                tree_type_ = OctoMapType::STANDARD;
                RCLCPP_INFO(node_->get_logger(), "Loaded standard octomap");
            }
            else
            {
                RCLCPP_WARN(node_->get_logger(), "Expected standard tree but cast failed");
                return;
            }
        }

        resolution_ = msg->octomap.resolution;
        last_update_time_ = node_->now();

        // Use bounding box from message if provided
        if (msg->has_bounding_box)
        {
            bbox_min_ = octomap::point3d(msg->bbx_min.x, msg->bbx_min.y, msg->bbx_min.z);
            bbox_max_ = octomap::point3d(msg->bbx_max.x, msg->bbx_max.y, msg->bbx_max.z);
            has_valid_bbox_ = true;
            RCLCPP_DEBUG(node_->get_logger(), "Octomap received: type=%s, resolution=%.4f, bbox=[%.2f,%.2f,%.2f] to [%.2f,%.2f,%.2f]",
                         msg->octomap.id.c_str(), resolution_, 
                         bbox_min_.x(), bbox_min_.y(), bbox_min_.z(),
                         bbox_max_.x(), bbox_max_.y(), bbox_max_.z());
        }
        else
        {
            has_valid_bbox_ = false;
            RCLCPP_DEBUG(node_->get_logger(), "Octomap received: type=%s, resolution=%.4f (no bounding box)", 
                         msg->octomap.id.c_str(), resolution_);
        }
    }

    std::shared_ptr<octomap::AbstractOcTree> OctoMapInterface::getTreeSnapshot() const
    {
        std::shared_lock lk(mtx_);
        return std::visit([](auto&& tree) -> std::shared_ptr<octomap::AbstractOcTree> {
            return tree;
        }, tree_);
    }

    OctoMapType OctoMapInterface::getTreeType() const
    {
        std::shared_lock lk(mtx_);
        return tree_type_;
    }

    bool OctoMapInterface::isSemanticTree() const
    {
        return getTreeType() == OctoMapType::SEMANTIC;
    }

    std::string OctoMapInterface::getTreeTypeString() const
    {
        std::shared_lock lk(mtx_);
        return tree_type_ == OctoMapType::SEMANTIC ? "SemanticOcTree" : "OcTree";
    }

    std::shared_ptr<octomap::OcTree> OctoMapInterface::getStandardTree() const
    {
        std::shared_lock lk(mtx_);
        if (tree_type_ == OctoMapType::STANDARD)
        {
            return std::get<std::shared_ptr<octomap::OcTree>>(tree_);
        }
        return nullptr;
    }

    std::shared_ptr<octomap::SemanticOcTree> OctoMapInterface::getSemanticTree() const
    {
        std::shared_lock lk(mtx_);
        if (tree_type_ == OctoMapType::SEMANTIC)
        {
            return std::get<std::shared_ptr<octomap::SemanticOcTree>>(tree_);
        }
        return nullptr;
    }

    bool OctoMapInterface::getVoxelSemantic(const octomap::point3d &point,
                                           int32_t &class_id,
                                           float &confidence) const
    {
        auto sem_tree = getSemanticTree();
        if (!sem_tree)
            return false;

        octomap::SemanticOcTreeNode *node = sem_tree->search(point);
        if (!node)
            return false;

        class_id = node->getClassId();
        confidence = node->getConfidence();
        return true;
    }

    std::vector<octomap::point3d> OctoMapInterface::findFrontiersByClass(
        int32_t class_id,
        float min_confidence,
        int min_unknown_neighbors,
        bool use_bbox) const
    {
        std::vector<octomap::point3d> filtered;
        
        auto sem_tree = getSemanticTree();
        if (!sem_tree)
        {
            RCLCPP_WARN(node_->get_logger(), "findFrontiersByClass called on non-semantic tree");
            return filtered;
        }

        // Get all frontiers first
        auto frontiers = findFrontiers(min_unknown_neighbors, use_bbox);

        // Filter by semantic class
        for (const auto &pt : frontiers)
        {
            octomap::SemanticOcTreeNode *node = sem_tree->search(pt);
            if (node && 
                node->getClassId() == class_id && 
                node->getConfidence() >= min_confidence)
            {
                filtered.push_back(pt);
            }
        }

        return filtered;
    }

    std::map<int32_t, size_t> OctoMapInterface::getSemanticClassCounts() const
    {
        std::map<int32_t, size_t> class_counts;
        
        auto sem_tree = getSemanticTree();
        if (!sem_tree)
        {
            return class_counts; // Return empty map for non-semantic trees
        }

        // Iterate through all leaf nodes and count occupied voxels by class
        for (auto it = sem_tree->begin_leafs(); it != sem_tree->end_leafs(); ++it)
        {
            if (sem_tree->isNodeOccupied(*it))
            {
                int32_t class_id = it->getClassId();
                class_counts[class_id]++;
            }
        }

        return class_counts;
    }

    std::vector<octomap::point3d> OctoMapInterface::findFrontiers(int min_unknown_neighbors,
                                                                  bool use_bbox) const
    {
        std::vector<octomap::point3d> frontiers;

        std::shared_lock lk(mtx_);
        
        double res = 0.0;
        
        // Check tree type and get resolution
        if (tree_type_ == OctoMapType::STANDARD)
        {
            auto std_tree = std::get<std::shared_ptr<octomap::OcTree>>(tree_);
            if (!std_tree)
                return frontiers;
            res = std_tree->getResolution();
        }
        else if (tree_type_ == OctoMapType::SEMANTIC)
        {
            auto sem_tree = std::get<std::shared_ptr<octomap::SemanticOcTree>>(tree_);
            if (!sem_tree)
                return frontiers;
            res = sem_tree->getResolution();
        }
        else
        {
            return frontiers;
        }

        // If using bbox, check if we have valid bounds
        if (use_bbox && !has_valid_bbox_)
        {
            RCLCPP_WARN(node_->get_logger(), "findFrontiers: use_bbox=true but no valid bounding box available");
            return frontiers;
        }

        // Lambda to check if point is inside bbox
        auto is_in_bbox = [&](const octomap::point3d &p) -> bool
        {
            if (!use_bbox)
                return true;
            return p.x() >= bbox_min_.x() && p.x() <= bbox_max_.x() &&
                   p.y() >= bbox_min_.y() && p.y() <= bbox_max_.y() &&
                   p.z() >= bbox_min_.z() && p.z() <= bbox_max_.z();
        };

        // Use std::visit to handle both tree types
        std::visit([&](auto&& tree) {
            if (!tree) return;
            
            // Lambdas for checking unknown neighbors
            auto is_unknown = [&](const octomap::point3d &p) -> bool
            {
                // If using bbox, don't count voxels outside bbox as unknown
                if (use_bbox && !is_in_bbox(p))
                {
                    return false;
                }
                return tree->search(p) == nullptr;
            };

            // Count unknown neighbors in 6-neighborhood
            auto count_unknown_6 = [&](const octomap::point3d &v) -> int
            {
                int unknown = 0;
                unknown += is_unknown(v + octomap::point3d(res, 0, 0));
                unknown += is_unknown(v + octomap::point3d(-res, 0, 0));
                unknown += is_unknown(v + octomap::point3d(0, res, 0));
                unknown += is_unknown(v + octomap::point3d(0, -res, 0));
                unknown += is_unknown(v + octomap::point3d(0, 0, res));
                unknown += is_unknown(v + octomap::point3d(0, 0, -res));
                return unknown;
            };

            // Search for frontiers (use bounding box if specified)
            if (!use_bbox)
            {
                for (auto it = tree->begin_leafs(); it != tree->end_leafs(); ++it)
                {
                    if (tree->isNodeOccupied(*it))
                        continue; // only free
                    const auto v = it.getCoordinate();
                    if (count_unknown_6(v) >= min_unknown_neighbors)
                        frontiers.push_back(v);
                }
            }
            else
            {
                for (auto it = tree->begin_leafs_bbx(bbox_min_, bbox_max_); it != tree->end_leafs_bbx(); ++it)
                {
                    if (tree->isNodeOccupied(*it))
                        continue;
                    const auto v = it.getCoordinate();
                    if (count_unknown_6(v) >= min_unknown_neighbors)
                        frontiers.push_back(v);
                }
            }
        }, tree_);

        return frontiers;
    }

    static inline double sqdist(const octomap::point3d &a, const octomap::point3d &b)
    {
        const double dx = a.x() - b.x();
        const double dy = a.y() - b.y();
        const double dz = a.z() - b.z();
        return dx * dx + dy * dy + dz * dz;
    }

    std::vector<Cluster> OctoMapInterface::kmeansCluster(const std::vector<octomap::point3d> &points,
                                                         int n_clusters,
                                                         int max_iters,
                                                         double tol) const
    {
        std::vector<Cluster> clusters;
        const int N = static_cast<int>(points.size());
        if (N == 0)
            return clusters;

        // Heuristic: ~1 cluster per 50 points
        if (n_clusters <= 0)
            n_clusters = std::max(1, N / 50);

        const int K = std::min(n_clusters, N);

        // Working data structures for k-means algorithm
        std::vector<int> labels(N, -1);
        std::vector<octomap::point3d> centers(K);
        std::vector<int> sizes(K, 0);

        // K-means++ initialization for faster convergence
        std::mt19937 rng(42); // Deterministic seed
        centers[0] = points[rng() % N];

        for (int k = 1; k < K; ++k)
        {
            std::vector<double> min_dists(N, std::numeric_limits<double>::infinity());

            // Update min distances to existing centers
            for (int i = 0; i < N; ++i)
            {
                double d = sqdist(points[i], centers[k - 1]);
                min_dists[i] = std::min(min_dists[i], d);
            }

            // Choose next center with probability proportional to squared distance
            double sum_dists = std::accumulate(min_dists.begin(), min_dists.end(), 0.0);
            std::uniform_real_distribution<double> dist(0.0, sum_dists);
            double target = dist(rng);

            double cumsum = 0.0;
            for (int i = 0; i < N; ++i)
            {
                cumsum += min_dists[i];
                if (cumsum >= target)
                {
                    centers[k] = points[i];
                    break;
                }
            }
        }

        std::vector<octomap::point3d> new_centers(K);
        std::vector<double> sum_x(K), sum_y(K), sum_z(K);

        // K-means iteration
        for (int iter = 0; iter < max_iters; ++iter)
        {
            // Reset accumulators
            std::fill(sizes.begin(), sizes.end(), 0);
            std::fill(sum_x.begin(), sum_x.end(), 0.0);
            std::fill(sum_y.begin(), sum_y.end(), 0.0);
            std::fill(sum_z.begin(), sum_z.end(), 0.0);

            // Assignment step: assign each point to nearest center
            for (int i = 0; i < N; ++i)
            {
                int best_k = 0;
                double best_d = sqdist(points[i], centers[0]);

                for (int k = 1; k < K; ++k)
                {
                    double d = sqdist(points[i], centers[k]);
                    if (d < best_d)
                    {
                        best_d = d;
                        best_k = k;
                    }
                }

                labels[i] = best_k;
                sizes[best_k]++;
                sum_x[best_k] += points[i].x();
                sum_y[best_k] += points[i].y();
                sum_z[best_k] += points[i].z();
            }

            // Update step: recompute centers and check convergence
            double max_shift = 0.0;
            for (int k = 0; k < K; ++k)
            {
                if (sizes[k] > 0)
                {
                    const double inv = 1.0 / static_cast<double>(sizes[k]);
                    octomap::point3d c(sum_x[k] * inv, sum_y[k] * inv, sum_z[k] * inv);
                    max_shift = std::max(max_shift, std::sqrt(sqdist(c, centers[k])));
                    centers[k] = c;
                }
            }

            if (max_shift < tol)
                break;
        }

        // Build output: create Cluster objects with their assigned points
        clusters.resize(K);
        for (int k = 0; k < K; ++k)
        {
            clusters[k].label = k;
            clusters[k].center = centers[k];
            clusters[k].size = sizes[k];
            clusters[k].points.reserve(sizes[k]);
        }

        // Populate points for each cluster
        for (int i = 0; i < N; ++i)
        {
            clusters[labels[i]].points.push_back(points[i]);
        }

        return clusters;
    }

    bool OctoMapInterface::isTreeAvailable() const
    {
        std::shared_lock lk(mtx_);
        return std::visit([](auto&& tree) { return tree != nullptr; }, tree_);
    }

    bool OctoMapInterface::getResolution(double &resolution_out) const
    {
        std::shared_lock lk(mtx_);
        bool has_tree = std::visit([](auto&& tree) { return tree != nullptr; }, tree_);
        if (!has_tree)
        {
            return false;
        }
        resolution_out = resolution_;
        return true;
    }

    bool OctoMapInterface::getBoundingBox(octomap::point3d &min_out,
                                          octomap::point3d &max_out) const
    {
        if (!has_valid_bbox_)
        {
            return false;
        }

        min_out = bbox_min_;
        max_out = bbox_max_;
        return true;
    }

    bool OctoMapInterface::isVoxelOccupied(const octomap::point3d &point) const
    {
        std::shared_lock lk(mtx_);
        
        return std::visit([&point](auto&& tree) -> bool {
            if (!tree)
                return false;
            auto *node = tree->search(point);
            return node != nullptr && tree->isNodeOccupied(node);
        }, tree_);
    }

    bool OctoMapInterface::searchOctree(const octomap::point3d &point,
                                        octomap::OcTreeNode *&node_out) const
    {
        std::shared_lock lk(mtx_);
        
        return std::visit([&point, &node_out](auto&& tree) -> bool {
            if (!tree)
            {
                node_out = nullptr;
                return false;
            }
            node_out = tree->search(point);
            return node_out != nullptr;
        }, tree_);
    }

    int OctoMapInterface::getOctreeDepth() const
    {
        std::shared_lock lk(mtx_);
        
        return std::visit([](auto&& tree) -> int {
            if (!tree)
                return 0;
            return static_cast<int>(tree->getTreeDepth());
        }, tree_);
    }

    rclcpp::Time OctoMapInterface::getLastUpdateTime() const
    {
        std::shared_lock lk(mtx_);
        return last_update_time_;
    }

}
