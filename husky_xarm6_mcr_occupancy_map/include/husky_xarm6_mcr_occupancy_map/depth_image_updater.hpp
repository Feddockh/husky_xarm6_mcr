#pragma once

#include "husky_xarm6_mcr_occupancy_map/occupancy_map_updater.hpp"
#include "husky_xarm6_mcr_occupancy_map/occupancy_map_monitor.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_ros/buffer.h>

#include <mutex>
#include <memory>
#include <string>

namespace husky_xarm6_mcr_occupancy_map
{

    /**
     * @brief Occupancy map updater that processes depth images.
     *
     * Subscribes to a depth image topic and camera info topic, performs ray tracing
     * to update the octree with free and occupied voxels based on depth measurements.
     * Supports both 32FC1 (float meters) and 16UC1 (uint16, scaled by depth_scale_) encodings.
     *
     * Key feature: When clear_no_return==true, rays with invalid depth (NaN/0)
     * are cast to max_range to mark free space even without valid returns (outdoor/no background).
     */
    class DepthImageUpdater : public OccupancyMapUpdater
    {
    public:
        DepthImageUpdater(
            const std::string &depth_topic,
            const std::string &info_topic,
            const OccupancyMapParameters &params);

        ~DepthImageUpdater() override = default;

        /**
         * @brief Initialize the updater with ROS node and TF buffer.
         * @return true if initialization successful
         */
        bool initialize(
            const rclcpp::Node::SharedPtr &node,
            const std::shared_ptr<tf2_ros::Buffer> &tf_buffer) override;

        void start() override;
        void stop() override;

    private:
        void infoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
        void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);

        static void quatToRot(
            const geometry_msgs::msg::Quaternion &q,
            double &r00, double &r01, double &r02,
            double &r10, double &r11, double &r12,
            double &r20, double &r21, double &r22);

        // ROS resources
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        rclcpp::Logger logger_;

        // Subscriptions
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;

        // Configuration
        std::string depth_topic_;
        std::string info_topic_;
        OccupancyMapParameters params_;

        // State
        bool active_{false};

        // Camera intrinsics (guarded by info_mtx_)
        std::mutex info_mtx_;
        bool have_info_{false};
        double fx_{0.0};
        double fy_{0.0};
        double cx_{0.0};
        double cy_{0.0};
        uint32_t cam_w_{0};
        uint32_t cam_h_{0};

        // Updater behavior parameters
        int stride_{2};              ///< pixel stride (2 = every other pixel)
        bool clear_no_return_{true}; ///< cast rays to max_range when depth invalid
        double depth_scale_{0.001};  ///< 16UC1 scale (mm->m)
        int queue_depth_{1};         ///< subscription queue
    };

} // namespace husky_xarm6_mcr_occupancy_map
