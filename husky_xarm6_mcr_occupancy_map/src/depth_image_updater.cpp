/**
 * @file depth_image_updater.cpp
 * @brief Depth-image octree updater with optional "no return clears to max_range"
 * This is currently not fully tested in the husky_xarm6_mcr_occupancy_map package,
 */

#include "husky_xarm6_mcr_occupancy_map/depth_image_updater.hpp"

#include <sensor_msgs/image_encodings.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <octomap/octomap.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace husky_xarm6_mcr_occupancy_map
{

    DepthImageUpdater::DepthImageUpdater(
        const std::string &depth_topic,
        const std::string &info_topic,
        const OccupancyMapParameters &params)
        : OccupancyMapUpdater("DepthImageUpdater"), node_(nullptr), tf_buffer_(nullptr), logger_(rclcpp::get_logger("DepthImageUpdater")), depth_topic_(depth_topic), info_topic_(info_topic), params_(params)
    {
    }

    bool DepthImageUpdater::initialize(
        const rclcpp::Node::SharedPtr &node,
        const std::shared_ptr<tf2_ros::Buffer> &tf_buffer)
    {
        node_ = node;
        tf_buffer_ = tf_buffer;
        logger_ = node_->get_logger();

        // These params live on the octomap_server node namespace (same as PointCloudUpdater style)
        stride_ = node_->declare_parameter<int>("depth_updater.stride", 2);
        clear_no_return_ = node_->declare_parameter<bool>("depth_updater.clear_no_return", true);
        depth_scale_ = node_->declare_parameter<double>("depth_updater.depth_scale", 0.001);
        queue_depth_ = node_->declare_parameter<int>("depth_updater.qos_depth", 1);

        active_ = false;
        have_info_ = false;

        RCLCPP_INFO(logger_, "DepthImageUpdater initialized:");
        RCLCPP_INFO(logger_, "  depth_topic: %s", depth_topic_.c_str());
        RCLCPP_INFO(logger_, "  info_topic:  %s", info_topic_.c_str());
        RCLCPP_INFO(logger_, "  stride: %d | clear_no_return: %s | depth_scale: %.6f | qos_depth: %d",
                    stride_, clear_no_return_ ? "true" : "false", depth_scale_, queue_depth_);
        return true;
    }

    void DepthImageUpdater::start()
    {
        if (active_)
        {
            RCLCPP_WARN(logger_, "DepthImageUpdater already active");
            return;
        }

        auto qos = rclcpp::QoS(rclcpp::KeepLast(std::max(1, queue_depth_)));
        qos.best_effort();

        info_sub_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
            info_topic_, qos,
            std::bind(&DepthImageUpdater::infoCallback, this, std::placeholders::_1));

        depth_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
            depth_topic_, qos,
            std::bind(&DepthImageUpdater::depthCallback, this, std::placeholders::_1));

        active_ = true;
        RCLCPP_INFO(logger_, "DepthImageUpdater started, subscribing to %s and %s",
                    depth_topic_.c_str(), info_topic_.c_str());
    }

    void DepthImageUpdater::stop()
    {
        if (!active_)
            return;

        depth_sub_.reset();
        info_sub_.reset();
        active_ = false;

        RCLCPP_INFO(logger_, "DepthImageUpdater stopped");
    }

    void DepthImageUpdater::infoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lk(info_mtx_);
        fx_ = msg->k[0];
        fy_ = msg->k[4];
        cx_ = msg->k[2];
        cy_ = msg->k[5];
        cam_w_ = msg->width;
        cam_h_ = msg->height;
        have_info_ = true;
    }

    void DepthImageUpdater::quatToRot(
        const geometry_msgs::msg::Quaternion &q,
        double &r00, double &r01, double &r02,
        double &r10, double &r11, double &r12,
        double &r20, double &r21, double &r22)
    {
        const double qx = q.x, qy = q.y, qz = q.z, qw = q.w;
        const double xx = qx * qx, yy = qy * qy, zz = qz * qz;
        const double xy = qx * qy, xz = qx * qz, yz = qy * qz;
        const double wx = qw * qx, wy = qw * qy, wz = qw * qz;

        r00 = 1.0 - 2.0 * (yy + zz);
        r01 = 2.0 * (xy - wz);
        r02 = 2.0 * (xz + wy);

        r10 = 2.0 * (xy + wz);
        r11 = 1.0 - 2.0 * (xx + zz);
        r12 = 2.0 * (yz - wx);

        r20 = 2.0 * (xz - wy);
        r21 = 2.0 * (yz + wx);
        r22 = 1.0 - 2.0 * (xx + yy);
    }

    void DepthImageUpdater::depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!tree_ || !active_)
            return;

        // Need intrinsics
        double fx, fy, cx, cy;
        uint32_t cw, ch;
        {
            std::lock_guard<std::mutex> lk(info_mtx_);
            if (!have_info_)
            {
                RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 2000, "DepthImageUpdater waiting for CameraInfo...");
                return;
            }
            fx = fx_;
            fy = fy_;
            cx = cx_;
            cy = cy_;
            cw = cam_w_;
            ch = cam_h_;
        }

        if (fx <= 0.0 || fy <= 0.0)
        {
            RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 2000, "Invalid intrinsics fx/fy");
            return;
        }

        if (msg->width == 0 || msg->height == 0)
        {
            return;
        }

        if ((msg->width != cw || msg->height != ch))
        {
            RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 2000,
                                 "Depth image size (%ux%u) != CameraInfo (%ux%u). Continuing anyway.",
                                 msg->width, msg->height, cw, ch);
        }

        const std::string sensor_frame = msg->header.frame_id;

        // TF lookup: map_frame <- sensor_frame
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
                logger_, *node_->get_clock(), 5000,
                "TF lookup failed: %s -> %s: %s",
                sensor_frame.c_str(), params_.map_frame.c_str(), ex.what());
            return;
        }

        // Sensor origin in map frame
        const auto &t = transform.transform.translation;
        octomap::point3d sensor_origin((float)t.x, (float)t.y, (float)t.z);

        // Rotation sensor->map
        double r00, r01, r02, r10, r11, r12, r20, r21, r22;
        quatToRot(transform.transform.rotation, r00, r01, r02, r10, r11, r12, r20, r21, r22);

        const bool is_32f = (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1 ||
                             msg->encoding == "32FC1");
        const bool is_16u = (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
                             msg->encoding == "16UC1");

        if (!is_32f && !is_16u)
        {
            RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 2000,
                                 "Unsupported depth encoding: %s (expected 32FC1 or 16UC1)", msg->encoding.c_str());
            return;
        }

        const int stride = std::max(1, stride_);
        const double minR = params_.min_range;
        const double maxR = params_.max_range;

        octomap::KeyRay key_ray;
        key_ray.reset();

        // Pointers into image data
        const float *data_f = is_32f ? reinterpret_cast<const float *>(msg->data.data()) : nullptr;
        const uint16_t *data_u16 = is_16u ? reinterpret_cast<const uint16_t *>(msg->data.data()) : nullptr;

        tree_->lockWrite();

        for (uint32_t v = 0; v < msg->height; v += (uint32_t)stride)
        {
            for (uint32_t u = 0; u < msg->width; u += (uint32_t)stride)
            {
                const size_t idx = (size_t)v * (size_t)msg->width + (size_t)u;

                double z_m = std::numeric_limits<double>::quiet_NaN();
                if (is_32f)
                {
                    z_m = (double)data_f[idx];
                }
                else
                {
                    const uint16_t raw = data_u16[idx];
                    z_m = (raw > 0) ? (double)raw * depth_scale_ : std::numeric_limits<double>::quiet_NaN();
                }

                const bool valid = std::isfinite(z_m) && (z_m > 0.0);
                double ray_len = 0.0;
                bool is_hit = false;

                if (valid)
                {
                    ray_len = z_m;
                    is_hit = true;
                }
                else
                {
                    if (!clear_no_return_)
                        continue;
                    ray_len = maxR;
                    is_hit = false;
                }

                if (ray_len < minR)
                    continue;
                if (ray_len > maxR)
                    ray_len = maxR;

                // Ray endpoint in SENSOR frame (optical pinhole):
                // xs = (u - cx) * Z / fx, ys = (v - cy) * Z / fy, zs = Z
                const double xs = ((double)u - cx) * ray_len / fx;
                const double ys = ((double)v - cy) * ray_len / fy;
                const double zs = ray_len;

                // Transform endpoint to MAP frame
                const double x_map = r00 * xs + r01 * ys + r02 * zs + t.x;
                const double y_map = r10 * xs + r11 * ys + r12 * zs + t.y;
                const double z_map = r20 * xs + r21 * ys + r22 * zs + t.z;

                octomap::point3d end_map((float)x_map, (float)y_map, (float)z_map);

                key_ray.reset();
                if (!tree_->computeRayKeys(sensor_origin, end_map, key_ray))
                {
                    continue;
                }

                // Update FREE along the ray (exclude endpoint)
                if (key_ray.size() > 0)
                {
                    const size_t n = key_ray.size();
                    const size_t free_n = (n > 0) ? (n - 1) : 0;

                    size_t k = 0;
                    for (auto it = key_ray.begin(); it != key_ray.end(); ++it, ++k)
                    {
                        if (k >= free_n)
                            break;
                        tree_->updateNode(*it, false /*occupied*/);
                    }

                    // Only mark occupied for real hits
                    if (is_hit)
                    {
                        tree_->updateNode(end_map, true /*occupied*/);
                    }
                }
            }
        }

        tree_->unlockWrite();

        tree_->triggerUpdateCallback();
    }

} // namespace husky_xarm6_mcr_occupancy_map
