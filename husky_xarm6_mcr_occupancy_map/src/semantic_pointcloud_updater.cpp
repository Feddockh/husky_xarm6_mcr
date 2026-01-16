/**
 * @file semantic_pointcloud_updater.cpp
 * @brief Implementation of semantic pointcloud updater
 */

#include "husky_xarm6_mcr_occupancy_map/semantic_pointcloud_updater.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

namespace husky_xarm6_mcr_occupancy_map
{

    SemanticPointCloudUpdater::SemanticPointCloudUpdater(
        const std::string &point_cloud_topic,
        const std::string &detections_topic,
        const OccupancyMapParameters &params)
        : OccupancyMapUpdater("SemanticPointCloudUpdater"),
          pointcloud_topic_(point_cloud_topic),
          detections_topic_(detections_topic),
          params_(params),
          active_(false),
          points_processed_(0)
    {
        // Initialize some common label mappings (can be extended)
        label_map_["person"] = 0;
        label_map_["object"] = 1;
        label_map_["obstacle"] = 2;
        // Add more as needed
    }

    bool SemanticPointCloudUpdater::initialize(
        const rclcpp::Node::SharedPtr &node,
        const std::shared_ptr<tf2_ros::Buffer> &tf_buffer)
    {
        node_ = node;
        tf_buffer_ = tf_buffer;

        // Create subscribers
        pc_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            pointcloud_topic_, rclcpp::SensorDataQoS(),
            std::bind(&SemanticPointCloudUpdater::pointCloudCallback, this, std::placeholders::_1));

        det_sub_ = node_->create_subscription<vision_msgs::msg::Detection3DArray>(
            detections_topic_, rclcpp::SensorDataQoS(),
            std::bind(&SemanticPointCloudUpdater::detectionsCallback, this, std::placeholders::_1));

        RCLCPP_INFO(node_->get_logger(), "SemanticPointCloudUpdater initialized");
        RCLCPP_INFO(node_->get_logger(), "  Point cloud topic: %s", pointcloud_topic_.c_str());
        RCLCPP_INFO(node_->get_logger(), "  Detections topic: %s", detections_topic_.c_str());

        return true;
    }

    void SemanticPointCloudUpdater::setSemanticMonitor(
        const std::shared_ptr<SemanticOccupancyMapMonitor> &monitor)
    {
        semantic_monitor_ = monitor;
    }

    void SemanticPointCloudUpdater::start()
    {
        active_ = true;
        RCLCPP_INFO(node_->get_logger(), "SemanticPointCloudUpdater started");
    }

    void SemanticPointCloudUpdater::stop()
    {
        active_ = false;
        RCLCPP_INFO(node_->get_logger(), "SemanticPointCloudUpdater stopped");
    }

    void SemanticPointCloudUpdater::pointCloudCallback(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!active_)
            return;

        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_pointcloud_ = msg;

        // Process if we have both pointcloud and detections
        if (latest_pointcloud_ && latest_detections_)
        {
            processSemanticPointCloud();
        }
    }

    void SemanticPointCloudUpdater::detectionsCallback(
        const vision_msgs::msg::Detection3DArray::SharedPtr msg)
    {
        if (!active_)
            return;

        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_detections_ = msg;

        // Process if we have both pointcloud and detections
        if (latest_pointcloud_ && latest_detections_)
        {
            processSemanticPointCloud();
        }
    }

    void SemanticPointCloudUpdater::processSemanticPointCloud()
    {
        if (!semantic_monitor_ || !semantic_monitor_->isActive())
            return;

        auto pc_msg = latest_pointcloud_;
        auto det_msg = latest_detections_;

        // Clear the cached messages to avoid reprocessing
        latest_pointcloud_.reset();
        latest_detections_.reset();

        // Transform to map frame
        std::string target_frame = params_.map_frame;
        std::string source_frame = pc_msg->header.frame_id;

        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform = tf_buffer_->lookupTransform(
                target_frame, source_frame, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                 "TF lookup failed: %s", ex.what());
            return;
        }

        // Get sensor origin in map frame
        octomap::point3d sensor_origin(
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z);

        // Create semantic pointcloud
        semantic_octomap_wrapper::SemanticPointCloud semantic_cloud;
        semantic_cloud.reserve(pc_msg->width * pc_msg->height);

        // Iterate through point cloud
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*pc_msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*pc_msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*pc_msg, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            // Skip invalid points
            if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z))
                continue;

            // Transform point to map frame
            geometry_msgs::msg::PointStamped point_in, point_out;
            point_in.header = pc_msg->header;
            point_in.point.x = *iter_x;
            point_in.point.y = *iter_y;
            point_in.point.z = *iter_z;

            tf2::doTransform(point_in, point_out, transform);

            octomap::point3d point(point_out.point.x, point_out.point.y, point_out.point.z);

            // Range filtering
            if (!isInRange(point, sensor_origin))
                continue;

            // Ground plane filtering
            if (params_.filter_ground_plane && isGroundPlane(point))
                continue;

            // Check which detection box this point belongs to
            int8_t label = -1; // Default: no label
            float confidence = 0.0f;

            for (const auto &detection : det_msg->detections)
            {
                // Detection bbox center is just a Pose, not PoseStamped
                // Transform it using the detection header
                geometry_msgs::msg::Pose box_pose = detection.bbox.center;
                
                if (detection.header.frame_id != target_frame)
                {
                    geometry_msgs::msg::PoseStamped pose_in, pose_out;
                    pose_in.header = detection.header;
                    pose_in.pose = detection.bbox.center;

                    try
                    {
                        auto det_transform = tf_buffer_->lookupTransform(
                            target_frame, pose_in.header.frame_id, tf2::TimePointZero);
                        tf2::doTransform(pose_in, pose_out, det_transform);
                        box_pose = pose_out.pose;
                    }
                    catch (const tf2::TransformException &ex)
                    {
                        continue; // Skip this detection
                    }
                }

                // Check if point is inside this bounding box
                if (isPointInBox(point, box_pose, detection.bbox.size))
                {
                    // Get label from detection
                    if (!detection.results.empty())
                    {
                        const auto &result = detection.results[0]; // Take highest confidence
                        label = getLabelId(result.hypothesis.class_id);
                        confidence = result.hypothesis.score;
                        break; // Use first matching box
                    }
                }
            }

            // Add point to semantic cloud
            semantic_cloud.push_back(point, label, confidence);
            points_processed_++;
        }

        // Insert semantic pointcloud into map
        auto semantic_map = semantic_monitor_->getSemanticMap();
        semantic_octomap_wrapper::SemanticUpdateParams update_params;
        update_params.confidence_boost = 0.01f;
        update_params.mismatch_penalty = 0.01f;

        semantic_map->insertPointCloudWithSemantics(
            semantic_cloud,
            sensor_origin,
            params_.max_range,
            false, // lazy_eval
            false, // discretize (disabled for compatibility)
            update_params);

        // Trigger update callback
        semantic_monitor_->triggerUpdateCallback();

        RCLCPP_DEBUG(node_->get_logger(),
                     "Processed %zu points (%zu total)",
                     semantic_cloud.size(), points_processed_);
    }

    bool SemanticPointCloudUpdater::isInRange(
        const octomap::point3d &point,
        const octomap::point3d &origin) const
    {
        double dist = point.distance(origin);
        return (dist >= params_.min_range && (params_.max_range < 0.0 || dist <= params_.max_range));
    }

    bool SemanticPointCloudUpdater::isGroundPlane(const octomap::point3d &point) const
    {
        return std::abs(point.z()) < params_.ground_distance_threshold;
    }

    bool SemanticPointCloudUpdater::isPointInBox(
        const octomap::point3d &point,
        const geometry_msgs::msg::Pose &box_pose,
        const geometry_msgs::msg::Vector3 &box_size) const
    {
        // Transform point to box-local coordinates
        // Simplified version: assumes axis-aligned boxes
        // For full implementation, would need to apply rotation transform

        double dx = std::abs(point.x() - box_pose.position.x);
        double dy = std::abs(point.y() - box_pose.position.y);
        double dz = std::abs(point.z() - box_pose.position.z);

        return (dx <= box_size.x / 2.0 &&
                dy <= box_size.y / 2.0 &&
                dz <= box_size.z / 2.0);
    }

    int8_t SemanticPointCloudUpdater::getLabelId(const std::string &class_id) const
    {
        auto it = label_map_.find(class_id);
        if (it != label_map_.end())
        {
            return it->second;
        }

        // Try to parse as integer
        try
        {
            return static_cast<int8_t>(std::stoi(class_id));
        }
        catch (...)
        {
            return -1; // Unknown label
        }
    }

} // namespace husky_xarm6_mcr_occupancy_map
