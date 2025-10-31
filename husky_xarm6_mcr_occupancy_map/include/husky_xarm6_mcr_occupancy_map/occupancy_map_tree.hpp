/**
 * @file occupancy_map_tree.hpp
 * @brief Thread-safe wrapper around octomap::OcTree
 *
 * This provides thread-safe access to the octree with read/write locks,
 * similar to MoveIt's OccMapTree but framework-agnostic.
 */

#pragma once

#include <octomap/octomap.h>
#include <functional>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>

namespace husky_xarm6_mcr_occupancy_map
{

    /**
     * @brief Thread-safe octomap wrapper with update callbacks
     *
     * Key features:
     * - Shared/exclusive locking for concurrent reads
     * - Update callbacks for notifying subscribers
     * - Compatible with octomap::OcTree interface
     * - Can be integrated with MoveIt2 or used standalone
     */
    class OccupancyMapTree : public octomap::OcTree
    {
    public:
        /**
         * @brief Construct with resolution
         * @param resolution Voxel size in meters
         */
        explicit OccupancyMapTree(double resolution);

        /**
         * @brief Load from file
         * @param filename Path to .bt or .ot file
         */
        explicit OccupancyMapTree(const std::string &filename);

        /**
         * @brief Destructor
         */
        virtual ~OccupancyMapTree() = default;

        // ========================================================================
        // Thread-safe locking
        // ========================================================================

        /**
         * @brief Acquire shared read lock (multiple readers allowed)
         */
        void lockRead();

        /**
         * @brief Release shared read lock
         */
        void unlockRead();

        /**
         * @brief Acquire exclusive write lock (only one writer)
         */
        void lockWrite();

        /**
         * @brief Release exclusive write lock
         */
        void unlockWrite();

        /**
         * @brief RAII helper for read locking
         */
        using ReadLock = std::shared_lock<std::shared_mutex>;

        /**
         * @brief RAII helper for write locking
         */
        using WriteLock = std::unique_lock<std::shared_mutex>;

        /**
         * @brief Get RAII read lock
         */
        ReadLock reading();

        /**
         * @brief Get RAII write lock
         */
        WriteLock writing();

        // ========================================================================
        // Update callbacks
        // ========================================================================

        /**
         * @brief Manually trigger update callback
         *
         * Call this after modifying the tree to notify subscribers
         * (e.g., MoveIt planning scene, visualization)
         */
        void triggerUpdateCallback();

        /**
         * @brief Set callback to be triggered on updates
         * @param callback Function to call when tree is updated
         */
        void setUpdateCallback(const std::function<void()> &callback);

        // ========================================================================
        // Statistics and queries
        // ========================================================================

        /**
         * @brief Get number of occupied nodes
         */
        size_t getNumOccupiedNodes() const;

        /**
         * @brief Get number of free nodes
         */
        size_t getNumFreeNodes() const;

        /**
         * @brief Get memory usage in bytes
         */
        size_t memoryUsage() const;

    private:
        mutable std::shared_mutex tree_mutex_;  ///< Thread-safety lock
        std::function<void()> update_callback_; ///< Callback on updates
    };

    // Smart pointer types
    using OccupancyMapTreePtr = std::shared_ptr<OccupancyMapTree>;
    using OccupancyMapTreeConstPtr = std::shared_ptr<const OccupancyMapTree>;

} // namespace husky_xarm6_mcr_occupancy_map
