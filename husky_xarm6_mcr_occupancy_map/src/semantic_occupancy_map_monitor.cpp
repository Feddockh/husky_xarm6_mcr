/**
 * @file semantic_occupancy_map_monitor.cpp
 * @brief Implementation of semantic occupancy map monitor
 */

#include "husky_xarm6_mcr_occupancy_map/semantic_occupancy_map_monitor.hpp"

namespace husky_xarm6_mcr_occupancy_map
{

    SemanticOccupancyMapMonitor::SemanticOccupancyMapMonitor(
        const OccupancyMapParameters &params,
        int32_t num_classes)
        : params_(params), active_(false)
    {
        semantic_map_ = std::make_shared<semantic_octomap_wrapper::SemanticOctoMap>(
            params_.resolution, num_classes);

        // Configure the underlying OcTree
        auto tree = semantic_map_->tree();
        tree->setProbHit(params_.prob_hit);
        tree->setProbMiss(params_.prob_miss);
        tree->setClampingThresMin(params_.clamp_min);
        tree->setClampingThresMax(params_.clamp_max);
        tree->setOccupancyThres(params_.occupancy_threshold);
    }

    std::shared_ptr<semantic_octomap_wrapper::SemanticOctoMap>
    SemanticOccupancyMapMonitor::getSemanticMap()
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        return semantic_map_;
    }

    std::shared_ptr<const semantic_octomap_wrapper::SemanticOctoMap>
    SemanticOccupancyMapMonitor::getSemanticMap() const
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        return semantic_map_;
    }

    std::shared_ptr<octomap::OcTree> SemanticOccupancyMapMonitor::getMapTree()
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        return semantic_map_->tree();
    }

    std::shared_ptr<const octomap::OcTree> SemanticOccupancyMapMonitor::getMapTree() const
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        return semantic_map_->tree();
    }

    void SemanticOccupancyMapMonitor::setUpdateCallback(UpdateCallback callback)
    {
        update_callback_ = callback;
    }

    void SemanticOccupancyMapMonitor::triggerUpdateCallback()
    {
        if (update_callback_)
        {
            update_callback_();
        }
    }

    void SemanticOccupancyMapMonitor::startMonitor()
    {
        active_ = true;
    }

    void SemanticOccupancyMapMonitor::stopMonitor()
    {
        active_ = false;
    }

} // namespace husky_xarm6_mcr_occupancy_map
