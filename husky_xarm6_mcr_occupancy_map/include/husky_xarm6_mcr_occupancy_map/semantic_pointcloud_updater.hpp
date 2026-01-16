/**
 * @file semantic_pointcloud_updater.hpp
 * @brief Semantic PointCloud2 processor for semantic octree updates
 */

#pragma once

#include "husky_xarm6_mcr_occupancy_map/occupancy_map_updater.hpp"
#include "husky_xarm6_mcr_occupancy_map/semantic_occupancy_map_monitor.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <mutex>

namespace husky_xarm6_mcr_occupancy_map
{

    /**
     * @brief Updates semantic octree from PointCloud2 and Detection3DArray messages
     *
     * Subscribes to point cloud and detection topics, associates semantic labels
     * with points based on 3D bounding boxes, and updates the semantic octomap.
     */
    class SemanticPointCloudUpdater : public OccupancyMapUpdater
    {
    public:
        /**
         * @brief Constructor
         * @param point_cloud_topic Topic name for PointCloud2
         * @param detections_topic Topic name for Detection3DArray
         * @param params Map parameters
         */
        SemanticPointCloudUpdater(
            const std::string &point_cloud_topic,
            const std::string &detections_topic,
            const OccupancyMapParameters &params);

        /**
         * @brief Destructor
         */
        virtual ~SemanticPointCloudUpdater() = default;

        /**
         * @brief Initialize with ROS node and TF
         */
        bool initialize(
            const rclcpp::Node::SharedPtr &node,
            const std::shared_ptr<tf2_ros::Buffer> &tf_buffer) override;

        /**
         * @brief Start subscribing to point cloud and detections
         */
        void start() override;

        /**
         * @brief Stop subscribing
         */
        void stop() override;

        /**
         * @brief Set the semantic monitor (overrides base class monitor)
         */
        void setSemanticMonitor(const std::shared_ptr<SemanticOccupancyMapMonitor> &monitor);

    private:
        /**
         * @brief Callback for point cloud messages
         */
        void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        /**
         * @brief Callback for detection messages
         */
        void detectionsCallback(const vision_msgs::msg::Detection3DArray::SharedPtr msg);

        /**
         * @brief Process combined pointcloud and detections
         */
        void processSemanticPointCloud();

        /**
         * @brief Check if point is within valid range
         */
        bool isInRange(const octomap::point3d &point, const octomap::point3d &origin) const;

        /**
         * @brief Filter ground plane (optional)
         */
        bool isGroundPlane(const octomap::point3d &point) const;

        /**
         * @brief Check if point is inside a 3D bounding box
         */
        bool isPointInBox(const octomap::point3d &point,
                          const geometry_msgs::msg::Pose &box_pose,
                          const geometry_msgs::msg::Vector3 &box_size) const;

        /**
         * @brief Get label ID from detection class
         */
        int8_t getLabelId(const std::string &class_id) const;

        std::string pointcloud_topic_;    ///< Point cloud topic
        std::string detections_topic_;    ///< Detections topic
        OccupancyMapParameters params_;   ///< Configuration
        
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;          ///< Point cloud subscriber
        rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr det_sub_;    ///< Detections subscriber
        
        std::shared_ptr<SemanticOccupancyMapMonitor> semantic_monitor_; ///< Semantic monitor
        
        // Synchronized data storage
        std::mutex data_mutex_;
        sensor_msgs::msg::PointCloud2::SharedPtr latest_pointcloud_;
        vision_msgs::msg::Detection3DArray::SharedPtr latest_detections_;
        
        bool active_;                 ///< Processing flag
        size_t points_processed_;     ///< Statistics
        
        // Label mapping (class_id -> label_id)
        std::unordered_map<std::string, int8_t> label_map_;
    };

} // namespace husky_xarm6_mcr_occupancy_map
