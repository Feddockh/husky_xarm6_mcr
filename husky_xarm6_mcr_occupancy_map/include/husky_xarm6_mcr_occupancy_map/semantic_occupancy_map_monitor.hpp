/**
 * @file semantic_occupancy_map_monitor.hpp
 * @brief Semantic occupancy map monitor using SemanticOctoMap
 */

#pragma once

#include "husky_xarm6_mcr_occupancy_map/occupancy_map_monitor.hpp"
#include <semantic_octomap/semantic_octomap.hpp>

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <memory>
#include <mutex>

namespace husky_xarm6_mcr_occupancy_map
{

    /**
     * @brief Manages semantic 3D occupancy map (octomap with semantic labels)
     *
     * Provides thread-safe access to semantic octomap for collision checking
     * and semantic querying.
     */
    class SemanticOccupancyMapMonitor
    {
    public:
        using UpdateCallback = std::function<void()>;

        /**
         * @brief Constructor
         * @param params Map configuration
         * @param num_classes Number of semantic classes
         */
        explicit SemanticOccupancyMapMonitor(const OccupancyMapParameters &params, 
                                             int32_t num_classes = 100);

        /**
         * @brief Destructor
         */
        ~SemanticOccupancyMapMonitor() = default;

        /**
         * @brief Get the semantic octomap (thread-safe read access)
         */
        std::shared_ptr<semantic_octomap_wrapper::SemanticOctoMap> getSemanticMap();
        std::shared_ptr<const semantic_octomap_wrapper::SemanticOctoMap> getSemanticMap() const;

        /**
         * @brief Get the underlying OcTree for MoveIt integration
         */
        std::shared_ptr<octomap::OcTree> getMapTree();
        std::shared_ptr<const octomap::OcTree> getMapTree() const;

        /**
         * @brief Set callback for map updates
         */
        void setUpdateCallback(UpdateCallback callback);

        /**
         * @brief Trigger update callback
         */
        void triggerUpdateCallback();

        /**
         * @brief Get map parameters
         */
        const OccupancyMapParameters &getParameters() const { return params_; }

        /**
         * @brief Start monitoring
         */
        void startMonitor();

        /**
         * @brief Stop monitoring
         */
        void stopMonitor();

        /**
         * @brief Check if monitor is active
         */
        bool isActive() const { return active_; }

    private:
        OccupancyMapParameters params_;
        std::shared_ptr<semantic_octomap_wrapper::SemanticOctoMap> semantic_map_;
        UpdateCallback update_callback_;
        mutable std::mutex map_mutex_;
        bool active_;
    };

} // namespace husky_xarm6_mcr_occupancy_map
