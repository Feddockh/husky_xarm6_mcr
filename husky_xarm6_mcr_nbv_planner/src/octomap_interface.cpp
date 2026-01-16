#include "husky_xarm6_mcr_nbv_planner/octomap_interface.hpp"

#include <octomap_msgs/conversions.h>
#include <cmath>
#include <limits>

namespace husky_xarm6_mcr_nbv_planner
{

    OctoMapInterface::OctoMapInterface(const rclcpp::Node::SharedPtr &node,
                                       const std::string &octomap_topic,
                                       bool transient_local) : node_(node)
    {
        rclcpp::QoS qos(1);
        if (transient_local)
            qos.transient_local();
        else
            qos.best_effort();

        sub_ = node_->create_subscription<octomap_msgs::msg::Octomap>(
            octomap_topic, qos,
            std::bind(&OctoMapInterface::onOctomap, this, std::placeholders::_1));

        RCLCPP_INFO(node_->get_logger(), "OctoMapInterface subscribing to %s", octomap_topic.c_str());
    }

    void OctoMapInterface::onOctomap(const octomap_msgs::msg::Octomap::SharedPtr msg)
    {
        std::unique_ptr<octomap::AbstractOcTree> abs(octomap_msgs::msgToMap(*msg));
        auto *tree = dynamic_cast<octomap::OcTree *>(abs.release());
        if (!tree)
        {
            RCLCPP_WARN(node_->get_logger(), "Received octomap, but msgToMap() was not an OcTree");
            return;
        }

        auto new_tree = std::shared_ptr<octomap::OcTree>(tree);

        {
            std::unique_lock lk(mtx_);
            tree_ = new_tree;
            resolution_ = tree_->getResolution();
        }
    }

    std::shared_ptr<octomap::OcTree> OctoMapInterface::getTreeSnapshot() const
    {
        std::shared_lock lk(mtx_);
        return tree_;
    }

    std::vector<octomap::point3d> OctoMapInterface::findFrontiers(int min_unknown_neighbors,
                                                                  bool use_bbox,
                                                                  const octomap::point3d &bbox_min,
                                                                  const octomap::point3d &bbox_max) const
    {
        std::vector<octomap::point3d> frontiers;

        // Get snapshot of the tree
        auto tree = getTreeSnapshot();
        if (!tree)
            return frontiers;
        const double res = tree->getResolution();

        // Lambdas for checking unknown neighbors
        auto is_unknown = [&](const octomap::point3d &p) -> bool
        {
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
            for (auto it = tree->begin_leafs_bbx(bbox_min, bbox_max); it != tree->end_leafs_bbx(); ++it)
            {
                if (tree->isNodeOccupied(*it))
                    continue;
                const auto v = it.getCoordinate();
                if (count_unknown_6(v) >= min_unknown_neighbors)
                    frontiers.push_back(v);
            }
        }

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

        // Heuristic: ~1 cluster per 40 points
        if (n_clusters <= 0)
            n_clusters = std::max(1, N / 40);

        const int K = std::min(n_clusters, N);

        // Working data structures for k-means algorithm
        std::vector<int> labels(N, -1);
        std::vector<octomap::point3d> centers(K);
        std::vector<int> sizes(K, 0);

        // Deterministic initialization: pick evenly spaced points
        for (int k = 0; k < K; ++k)
            centers[k] = points[(k * N) / K];

        std::vector<octomap::point3d> new_centers(K, octomap::point3d(0, 0, 0));

        // K-means iteration
        for (int iter = 0; iter < max_iters; ++iter)
        {
            std::fill(sizes.begin(), sizes.end(), 0);
            for (int k = 0; k < K; ++k)
                new_centers[k] = octomap::point3d(0, 0, 0);

            // Assignment step: assign each point to nearest center
            for (int i = 0; i < N; ++i)
            {
                int best_k = 0;
                double best_d = std::numeric_limits<double>::infinity();
                for (int k = 0; k < K; ++k)
                {
                    double d = sqdist(points[i], centers[k]);
                    if (d < best_d)
                    {
                        best_d = d;
                        best_k = k;
                    }
                }
                labels[i] = best_k;
                sizes[best_k] += 1;
                new_centers[best_k] += points[i];
            }

            // Update step: recompute centers and check convergence
            double max_shift = 0.0;
            for (int k = 0; k < K; ++k)
            {
                if (sizes[k] > 0)
                {
                    const double inv = 1.0 / static_cast<double>(sizes[k]);
                    octomap::point3d c(new_centers[k].x() * inv, 
                                      new_centers[k].y() * inv, 
                                      new_centers[k].z() * inv);
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

}
