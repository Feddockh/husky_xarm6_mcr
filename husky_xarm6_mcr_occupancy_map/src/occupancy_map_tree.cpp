/**
 * @file occupancy_map_tree.cpp
 * @brief Implementation of thread-safe octomap wrapper
 */

#include "husky_xarm6_mcr_occupancy_map/occupancy_map_tree.hpp"
#include <octomap/octomap.h>

namespace husky_xarm6_mcr_occupancy_map
{

    OccupancyMapTree::OccupancyMapTree(double resolution)
        : octomap::OcTree(resolution)
    {
    }

    OccupancyMapTree::OccupancyMapTree(const std::string &filename)
        : octomap::OcTree(filename)
    {
    }

    // ============================================================================
    // Thread-safe locking
    // ============================================================================

    void OccupancyMapTree::lockRead()
    {
        tree_mutex_.lock_shared();
    }

    void OccupancyMapTree::unlockRead()
    {
        tree_mutex_.unlock_shared();
    }

    void OccupancyMapTree::lockWrite()
    {
        tree_mutex_.lock();
    }

    void OccupancyMapTree::unlockWrite()
    {
        tree_mutex_.unlock();
    }

    OccupancyMapTree::ReadLock OccupancyMapTree::reading()
    {
        return ReadLock(tree_mutex_);
    }

    OccupancyMapTree::WriteLock OccupancyMapTree::writing()
    {
        return WriteLock(tree_mutex_);
    }

    // ============================================================================
    // Update callbacks
    // ============================================================================

    void OccupancyMapTree::triggerUpdateCallback()
    {
        if (update_callback_)
        {
            update_callback_();
        }
    }

    void OccupancyMapTree::setUpdateCallback(const std::function<void()> &callback)
    {
        update_callback_ = callback;
    }

    // ============================================================================
    // Statistics and queries
    // ============================================================================

    size_t OccupancyMapTree::getNumOccupiedNodes() const
    {
        size_t count = 0;
        for (auto it = this->begin_leafs(); it != this->end_leafs(); ++it)
        {
            if (this->isNodeOccupied(*it))
            {
                count++;
            }
        }
        return count;
    }

    size_t OccupancyMapTree::getNumFreeNodes() const
    {
        size_t count = 0;
        for (auto it = this->begin_leafs(); it != this->end_leafs(); ++it)
        {
            if (!this->isNodeOccupied(*it))
            {
                count++;
            }
        }
        return count;
    }

    size_t OccupancyMapTree::memoryUsage() const
    {
        return this->memoryUsage();
    }

} // namespace husky_xarm6_mcr_occupancy_map
