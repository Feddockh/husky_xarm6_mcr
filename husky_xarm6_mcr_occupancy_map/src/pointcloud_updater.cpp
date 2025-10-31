/**
 * @file pointcloud_updater.cpp
 * @brief Implementation of PointCloud2 octree updater
 */

#include "husky_xarm6_mcr_occupancy_map/pointcloud_updater.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point.hpp>

namespace husky_xarm6_mcr_occupancy_map
{

    PointCloudUpdater::PointCloudUpdater(
        const std::string &point_cloud_topic,
        const OccupancyMapParameters &params)
        : OccupancyMapUpdater("PointCloudUpdater"), topic_(point_cloud_topic), params_(params), active_(false), points_processed_(0)
    {
    }

    bool PointCloudUpdater::initialize(
        const rclcpp::Node::SharedPtr &node,
        const std::shared_ptr<tf2_ros::Buffer> &tf_buffer)
    {
        node_ = node;
        tf_buffer_ = tf_buffer;
        logger_ = node_->get_logger();

        RCLCPP_INFO(logger_, "PointCloudUpdater initialized for topic: %s", topic_.c_str());
        return true;
    }

    void PointCloudUpdater::start()
    {
        if (active_)
        {
            RCLCPP_WARN(logger_, "PointCloudUpdater already active");
            return;
        }

        // Create subscription with best-effort QoS (typical for sensors)
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
        qos.best_effort();

        sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic_,
            qos,
            std::bind(&PointCloudUpdater::pointCloudCallback, this, std::placeholders::_1));

        active_ = true;
        RCLCPP_INFO(logger_, "PointCloudUpdater started, subscribing to %s", topic_.c_str());
    }

    void PointCloudUpdater::stop()
    {
        if (!active_)
        {
            return;
        }

        sub_.reset();
        active_ = false;
        RCLCPP_INFO(logger_, "PointCloudUpdater stopped");
    }

    void PointCloudUpdater::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!tree_ || !active_)
        {
            return;
        }

        const std::string &sensor_frame = msg->header.frame_id;

        // 1. Get transform from sensor frame to map frame
        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform = tf_buffer_->lookupTransform(
                params_.map_frame,
                sensor_frame,
                msg->header.stamp,
                rclcpp::Duration::from_seconds(0.1));
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(
                logger_,
                *node_->get_clock(),
                5000,
                "TF lookup failed: %s -> %s: %s",
                sensor_frame.c_str(),
                params_.map_frame.c_str(),
                ex.what());
            return;
        }

        // 2. Get sensor origin in map frame
        octomap::point3d sensor_origin(
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z);

        // 3. Parse point cloud and transform to map frame
        std::vector<octomap::point3d> points;
        points.reserve(msg->width * msg->height);

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        // Extract transform as matrix for efficiency
        const auto &t = transform.transform.translation;
        const auto &r = transform.transform.rotation;

        // Convert quaternion to rotation matrix (simplified)
        double qx = r.x, qy = r.y, qz = r.z, qw = r.w;
        double xx = qx * qx, yy = qy * qy, zz = qz * qz;
        double xy = qx * qy, xz = qx * qz, yz = qy * qz;
        double wx = qw * qx, wy = qw * qy, wz = qw * qz;

        double r00 = 1.0 - 2.0 * (yy + zz);
        double r01 = 2.0 * (xy - wz);
        double r02 = 2.0 * (xz + wy);
        double r10 = 2.0 * (xy + wz);
        double r11 = 1.0 - 2.0 * (xx + zz);
        double r12 = 2.0 * (yz - wx);
        double r20 = 2.0 * (xz - wy);
        double r21 = 2.0 * (yz + wx);
        double r22 = 1.0 - 2.0 * (xx + yy);

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            // Skip NaN/Inf points
            if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z))
            {
                continue;
            }

            // Transform point to map frame
            double px = *iter_x;
            double py = *iter_y;
            double pz = *iter_z;

            double x_map = r00 * px + r01 * py + r02 * pz + t.x;
            double y_map = r10 * px + r11 * py + r12 * pz + t.y;
            double z_map = r20 * px + r21 * py + r22 * pz + t.z;

            octomap::point3d point(x_map, y_map, z_map);

            // Filter by range
            if (!isInRange(point, sensor_origin))
            {
                continue;
            }

            // Optional: filter ground plane
            if (params_.filter_ground_plane && isGroundPlane(point))
            {
                continue;
            }

            points.push_back(point);
        }

        if (points.empty())
        {
            return;
        }

        // 4. Update octree with ray casting (thread-safe)
        tree_->lockWrite();

        for (const auto &point : points)
        {
            // insertRay marks voxels along ray as free and endpoint as occupied
            tree_->insertRay(sensor_origin, point, params_.max_range);
        }

        tree_->unlockWrite();

        // 5. Trigger update callback
        tree_->triggerUpdateCallback();

        // Statistics
        points_processed_ += points.size();

        RCLCPP_DEBUG(
            logger_,
            "Processed %zu points from %s (total: %zu)",
            points.size(),
            sensor_frame.c_str(),
            points_processed_);
    }

    bool PointCloudUpdater::isInRange(const octomap::point3d &point, const octomap::point3d &origin) const
    {
        double distance = (point - origin).norm();
        return (distance >= params_.min_range && distance <= params_.max_range);
    }

    bool PointCloudUpdater::isGroundPlane(const octomap::point3d &point) const
    {
        // Simple ground filter: check if z coordinate is near ground level
        // Using ground_distance_threshold as both the ground height and tolerance
        return std::abs(point.z()) < params_.ground_distance_threshold;
    }

} // namespace husky_xarm6_mcr_occupancy_map
