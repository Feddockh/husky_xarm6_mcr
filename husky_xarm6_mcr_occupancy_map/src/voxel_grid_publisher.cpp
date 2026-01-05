/**
 * @file voxel_grid_publisher.cpp
 * @brief Implementation of simple voxel grid publisher
 */

#include "husky_xarm6_mcr_occupancy_map/voxel_grid_publisher.hpp"

namespace husky_xarm6_mcr_occupancy_map
{

VoxelGridPublisher::VoxelGridPublisher(
    const rclcpp::Node::SharedPtr &node,
    const std::string &topic)
    : node_(node), logger_(node_->get_logger())
{
    pub_ = node_->create_publisher<husky_xarm6_mcr_occupancy_map::msg::VoxelGrid>(
        topic, rclcpp::QoS(1).transient_local());
    
    RCLCPP_INFO(logger_, "VoxelGridPublisher created on topic: %s", topic.c_str());
}

void VoxelGridPublisher::publishVoxelGrid(
    const OccupancyMapTreePtr &tree,
    const std::string &frame_id)
{
    if (!tree)
    {
        RCLCPP_WARN(logger_, "Cannot publish - tree is null");
        return;
    }

    auto msg = husky_xarm6_mcr_occupancy_map::msg::VoxelGrid();
    msg.header.stamp = node_->now();
    msg.header.frame_id = frame_id;

    // Read-lock the tree
    auto lock = tree->reading();

    // Get tree parameters
    msg.resolution = tree->getResolution();

    // Get bounding box
    double min_x, min_y, min_z, max_x, max_y, max_z;
    tree->getMetricMin(min_x, min_y, min_z);
    tree->getMetricMax(max_x, max_y, max_z);

    msg.min_x = min_x;
    msg.min_y = min_y;
    msg.min_z = min_z;
    msg.max_x = max_x;
    msg.max_y = max_y;
    msg.max_z = max_z;

    // Origin (use minimum corner)
    msg.origin_x = min_x;
    msg.origin_y = min_y;
    msg.origin_z = min_z;

    // Extract leaf voxels
    // Reserve space for efficiency
    size_t estimated_size = tree->getNumLeafNodes();
    msg.voxel_x.reserve(estimated_size);
    msg.voxel_y.reserve(estimated_size);
    msg.voxel_z.reserve(estimated_size);
    msg.voxel_state.reserve(estimated_size);

    // Iterate through all leaf nodes
    for (auto it = tree->begin_leafs(); it != tree->end_leafs(); ++it)
    {
        // Get voxel center
        octomap::point3d coord = it.getCoordinate();
        
        msg.voxel_x.push_back(static_cast<float>(coord.x()));
        msg.voxel_y.push_back(static_cast<float>(coord.y()));
        msg.voxel_z.push_back(static_cast<float>(coord.z()));

        // Determine state: -1=free, 0=unknown, 1=occupied
        int8_t state;
        if (tree->isNodeOccupied(*it))
        {
            state = 1; // occupied
        }
        else
        {
            state = -1; // free
        }
        msg.voxel_state.push_back(state);
    }

    RCLCPP_INFO(logger_, 
        "Publishing voxel grid: %zu voxels, resolution=%.3fm, bbox=[%.2f,%.2f,%.2f] to [%.2f,%.2f,%.2f]",
        msg.voxel_x.size(), msg.resolution, min_x, min_y, min_z, max_x, max_y, max_z);

    pub_->publish(msg);
}

} // namespace husky_xarm6_mcr_occupancy_map
