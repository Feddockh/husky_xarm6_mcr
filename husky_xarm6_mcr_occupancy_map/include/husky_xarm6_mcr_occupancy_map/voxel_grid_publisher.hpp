/**
 * @file voxel_grid_publisher.hpp
 * @brief Simple voxel grid publisher for NBV planning
 * 
 * This publisher extracts voxel data from the octomap and publishes it
 * in a simple array format that's easy to use in Python and extend with
 * semantic labels later.
 */

#ifndef HUSKY_XARM6_MCR_OCCUPANCY_MAP__VOXEL_GRID_PUBLISHER_HPP_
#define HUSKY_XARM6_MCR_OCCUPANCY_MAP__VOXEL_GRID_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <octomap/octomap.h>
#include "husky_xarm6_mcr_occupancy_map/occupancy_map_tree.hpp"
#include "husky_xarm6_mcr_occupancy_map/msg/voxel_grid.hpp"

namespace husky_xarm6_mcr_occupancy_map
{

/**
 * @brief Publishes simple voxel grid for NBV planning
 */
class VoxelGridPublisher
{
public:
    VoxelGridPublisher(
        const rclcpp::Node::SharedPtr &node,
        const std::string &topic = "voxel_grid");

    /**
     * @brief Extract and publish voxels from octomap
     * @param tree The octomap tree
     * @param frame_id Map frame
     */
    void publishVoxelGrid(
        const OccupancyMapTreePtr &tree,
        const std::string &frame_id);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<husky_xarm6_mcr_occupancy_map::msg::VoxelGrid>::SharedPtr pub_;
    rclcpp::Logger logger_;
};

} // namespace husky_xarm6_mcr_occupancy_map

#endif // HUSKY_XARM6_MCR_OCCUPANCY_MAP__VOXEL_GRID_PUBLISHER_HPP_
