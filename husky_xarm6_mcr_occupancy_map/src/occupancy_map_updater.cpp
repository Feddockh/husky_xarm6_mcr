/**
 * @file occupancy_map_updater.cpp
 * @brief Implementation of base updater class
 */

#include "husky_xarm6_mcr_occupancy_map/occupancy_map_updater.hpp"
#include "husky_xarm6_mcr_occupancy_map/occupancy_map_monitor.hpp"

namespace husky_xarm6_mcr_occupancy_map
{

    OccupancyMapUpdater::OccupancyMapUpdater(const std::string &type)
        : type_(type), monitor_(nullptr), logger_(rclcpp::get_logger("occupancy_map_updater"))
    {
    }

    OccupancyMapUpdater::~OccupancyMapUpdater() = default;

    void OccupancyMapUpdater::setMonitor(OccupancyMapMonitor *monitor)
    {
        monitor_ = monitor;
        if (monitor_)
        {
            tree_ = monitor_->getOcTreePtr();
        }
    }

} // namespace husky_xarm6_mcr_occupancy_map
