#include "husky_xarm6_mcr_nbv_planner/viewpoint_generation.hpp"
#include <tf2_eigen/tf2_eigen.hpp>
#include <cmath>
#include <random>
#include <iostream>

namespace husky_xarm6_mcr_nbv_planner
{

    std::optional<std::vector<double>> computeViewpointJointAngles(
        const std::shared_ptr<MoveItInterface> &moveit_interface,
        Viewpoint &viewpoint,
        const std::string &camera_link,
        double ik_timeout,
        int ik_attempts)
    {
        if (!moveit_interface)
        {
            std::cerr << "MoveItInterface is null" << std::endl;
            return std::nullopt;
        }

        // ------------------------------------------------------------
        // 1) Construct camera pose from viewpoint (world frame)
        // ------------------------------------------------------------
        geometry_msgs::msg::Pose cam_pose_world;
        cam_pose_world.position.x = viewpoint.position.x();
        cam_pose_world.position.y = viewpoint.position.y();
        cam_pose_world.position.z = viewpoint.position.z();

        // Assumes viewpoint.orientation = [x, y, z, w]
        cam_pose_world.orientation.x = viewpoint.orientation[0];
        cam_pose_world.orientation.y = viewpoint.orientation[1];
        cam_pose_world.orientation.z = viewpoint.orientation[2];
        cam_pose_world.orientation.w = viewpoint.orientation[3];

        // ------------------------------------------------------------
        // 2) Convert camera pose → end-effector pose
        // ------------------------------------------------------------
        geometry_msgs::msg::Pose target_ee_pose;
        if (!moveit_interface->cameraPoseToEEPose(
                cam_pose_world, camera_link, target_ee_pose))
        {
            std::cerr << "Failed to convert camera pose to end-effector pose "
                      << "(camera_link = " << camera_link << ")" << std::endl;
            return std::nullopt;
        }

        // ------------------------------------------------------------
        // 3) Get current joint state as IK seed
        // ------------------------------------------------------------
        std::vector<double> seed_positions;
        if (!moveit_interface->getCurrentJointAngles(seed_positions))
        {
            std::cerr << "Failed to get current joint angles" << std::endl;
            return std::nullopt;
        }

        // ------------------------------------------------------------
        // 4) Solve IK
        // ------------------------------------------------------------
        std::vector<double> joint_angles = moveit_interface->computeIK(
            seed_positions,
            target_ee_pose,
            ik_timeout,
            ik_attempts);

        if (joint_angles.empty())
        {
            std::cerr << "IK failed for viewpoint at ["
                      << viewpoint.position.x() << ", "
                      << viewpoint.position.y() << ", "
                      << viewpoint.position.z() << "]" << std::endl;
            return std::nullopt;
        }

        // ------------------------------------------------------------
        // 5) Validate IK solution via FK
        // ------------------------------------------------------------
        Eigen::Isometry3d target_ee_eigen;
        tf2::fromMsg(target_ee_pose, target_ee_eigen);

        double pos_error = 0.0;
        double angular_error = 0.0;
        if (!moveit_interface->validateIKSolution(
                joint_angles,
                target_ee_eigen,
                pos_error,
                angular_error))
        {
            std::cerr << "Warning: IK solution validation failed. "
                      << "Position error: " << pos_error << " m, "
                      << "Angular error: "
                      << (angular_error * 180.0 / M_PI) << " deg" << std::endl;
        }

        // ------------------------------------------------------------
        // 6) Store solution in viewpoint
        // ------------------------------------------------------------
        viewpoint.joint_angles = joint_angles;

        return joint_angles;
    }

    std::vector<Viewpoint> computeSphericalCap(
        const Eigen::Vector3d &position,
        const std::array<double, 4> &orientation,
        double radius,
        double angular_resolution,
        double max_theta,
        bool look_at_center,
        bool use_positive_z)
    {
        if (radius <= 0.0)
        {
            throw std::invalid_argument("radius must be > 0");
        }
        if (angular_resolution <= 0.0 || max_theta < 0.0)
        {
            throw std::invalid_argument("angular_resolution must be > 0 and max_theta >= 0");
        }

        std::vector<Viewpoint> viewpoints;

        const Eigen::Quaterniond base_quat = geometry_utils::arrayToEigenQuat(orientation);
        const Eigen::Matrix3d R_base = base_quat.toRotationMatrix();
        const Eigen::Vector3d cap_x = R_base.col(0);
        const Eigen::Vector3d cap_y = R_base.col(1);
        const Eigen::Vector3d cap_z = R_base.col(2);

        const double z_sign = use_positive_z ? 1.0 : -1.0;
        const Eigen::Vector3d cap_axis = z_sign * cap_z; // <-- FIX

        std::vector<double> thetas = {0.0};
        for (double t = angular_resolution; t <= max_theta + 1e-12; t += angular_resolution)
        {
            thetas.push_back(t);
        }

        for (size_t ring_idx = 0; ring_idx < thetas.size(); ++ring_idx)
        {
            const double theta = thetas[ring_idx];

            if (theta == 0.0)
            {
                const Eigen::Vector3d local(0.0, 0.0, z_sign * radius);
                const Eigen::Vector3d p = position + local.x() * cap_x + local.y() * cap_y + local.z() * cap_z;

                Eigen::Vector3d direction = look_at_center ? (position - p) : (p - position);
                const double n = direction.norm();
                if (n < 1e-12)
                    continue;
                direction /= n;

                const auto quat_delta = geometry_utils::quatFromTwoVectors(cap_axis, direction); // <-- FIX
                const Eigen::Quaterniond q_delta = geometry_utils::arrayToEigenQuat(quat_delta);
                const Eigen::Quaterniond q_world = q_delta * base_quat;

                Viewpoint vp(p, geometry_utils::eigenQuatToArray(q_world));
                vp.target = position;
                viewpoints.push_back(vp);
                continue;
            }

            const double ring_circum = 2.0 * M_PI * std::sin(theta);
            const int m = std::max(1, static_cast<int>(std::ceil(ring_circum / angular_resolution)));
            const double phi_offset = (ring_idx % 2 == 1) ? (M_PI / m) : 0.0;

            for (int k = 0; k < m; ++k)
            {
                const double phi = (2.0 * M_PI * k) / m + phi_offset;

                const double local_x = radius * std::sin(theta) * std::cos(phi);
                const double local_y = radius * std::sin(theta) * std::sin(phi);
                const double local_z = z_sign * radius * std::cos(theta);

                const Eigen::Vector3d p = position + local_x * cap_x + local_y * cap_y + local_z * cap_z;

                Eigen::Vector3d direction = look_at_center ? (position - p) : (p - position);
                const double n = direction.norm();
                if (n < 1e-12)
                    continue;
                direction /= n;

                const auto quat_delta = geometry_utils::quatFromTwoVectors(cap_axis, direction); // <-- FIX
                const Eigen::Quaterniond q_delta = geometry_utils::arrayToEigenQuat(quat_delta);
                const Eigen::Quaterniond q_world = q_delta * base_quat;

                Viewpoint vp(p, geometry_utils::eigenQuatToArray(q_world));
                vp.target = position;
                viewpoints.push_back(vp);
            }
        }

        return viewpoints;
    }

    std::vector<Viewpoint> generatePlanarSphericalCapCandidates(
        const Eigen::Vector3d &position,
        const std::array<double, 4> &orientation,
        double half_extent,
        double spatial_resolution,
        double max_theta,
        double angular_resolution)
    {
        // Extract basis vectors from orientation
        const Eigen::Quaterniond base_quat = geometry_utils::arrayToEigenQuat(orientation);
        const Eigen::Matrix3d R_base = base_quat.toRotationMatrix();
        const Eigen::Vector3d plane_x = R_base.col(0);
        const Eigen::Vector3d plane_y = R_base.col(1);
        const Eigen::Vector3d normal = R_base.col(2);

        // Generate grid of positions on the plane
        std::vector<Eigen::Vector3d> grid_positions;
        for (double x = -half_extent; x <= half_extent + spatial_resolution / 2; x += spatial_resolution)
        {
            for (double y = -half_extent; y <= half_extent + spatial_resolution / 2; y += spatial_resolution)
            {
                const Eigen::Vector3d pos = position + x * plane_x + y * plane_y;
                grid_positions.push_back(pos);
            }
        }

        // Generate orientation variations
        std::vector<std::array<double, 4>> orientations;

        if (max_theta == 0 || angular_resolution == 0)
        {
            // No variations, just use base orientation
            orientations.push_back(orientation);
        }
        else
        {
            // Get orientations from spherical cap (dummy position)
            const auto cap_viewpoints = computeSphericalCap(
                Eigen::Vector3d::Zero(), orientation, 1.0,
                angular_resolution, max_theta, false, true);

            for (const auto &vp : cap_viewpoints)
            {
                orientations.push_back(vp.orientation);
            }
        }

        // Create viewpoints for each position with each orientation
        std::vector<Viewpoint> viewpoints;
        for (const auto &pos : grid_positions)
        {
            for (const auto &orient : orientations)
            {
                Viewpoint vp(pos, orient);
                vp.target = pos + normal;
                viewpoints.push_back(vp);
            }
        }

        return viewpoints;
    }

    std::vector<Viewpoint> sampleViewsFromHemisphere(
        const Eigen::Vector3d &center,
        const std::array<double, 4> &base_orientation,
        double min_radius,
        double max_radius,
        int num_samples,
        bool use_positive_z,
        double z_bias_sigma,
        double min_distance,
        int max_attempts)
    {
        if (min_radius < 0)
        {
            throw std::invalid_argument("min_radius must be non-negative");
        }
        if (max_radius < min_radius)
        {
            throw std::invalid_argument("max_radius must be >= min_radius");
        }
        if (num_samples <= 0)
        {
            throw std::invalid_argument("num_samples must be positive");
        }
        if (z_bias_sigma <= 0)
        {
            throw std::invalid_argument("z_bias_sigma must be positive");
        }
        if (min_distance < 0)
        {
            throw std::invalid_argument("min_distance must be non-negative");
        }

        const bool sample_surface = std::abs(max_radius - min_radius) < 1e-9;

        // Extract basis vectors
        const Eigen::Quaterniond base_quat = geometry_utils::arrayToEigenQuat(base_orientation);
        const Eigen::Matrix3d R_base = base_quat.toRotationMatrix();
        const Eigen::Vector3d cap_x = R_base.col(0);
        const Eigen::Vector3d cap_y = R_base.col(1);
        const Eigen::Vector3d cap_z = R_base.col(2);

        const double z_sign = use_positive_z ? 1.0 : -1.0;

        // Random number generators
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> uniform_01(0.0, 1.0);
        std::uniform_real_distribution<> uniform_phi(0.0, 2.0 * M_PI);
        std::normal_distribution<> gaussian_theta(0.0, z_bias_sigma);

        std::vector<Viewpoint> viewpoints;
        std::vector<Eigen::Vector3d> positions;

        int attempts = 0;
        while (viewpoints.size() < static_cast<size_t>(num_samples) && attempts < max_attempts)
        {
            attempts++;

            // Sample radius
            double r;
            if (sample_surface)
            {
                r = min_radius;
            }
            else
            {
                const double u = uniform_01(gen);
                const double r_cubed = min_radius * min_radius * min_radius +
                                       u * (max_radius * max_radius * max_radius - min_radius * min_radius * min_radius);
                r = std::cbrt(r_cubed);
            }

            // Gaussian sampling for theta, clamped to [0, pi/2]
            double theta = std::abs(gaussian_theta(gen));
            theta = std::min(theta, M_PI / 2.0);

            // Uniform azimuthal angle
            const double phi = uniform_phi(gen);

            // Convert to Cartesian in local frame
            const double local_x = r * std::sin(theta) * std::cos(phi);
            const double local_y = r * std::sin(theta) * std::sin(phi);
            const double local_z = z_sign * r * std::cos(theta);

            // Transform to world
            const Eigen::Vector3d pos = center +
                                        local_x * cap_x + local_y * cap_y + local_z * cap_z;

            // Check minimum distance constraint
            if (min_distance > 0 && !positions.empty())
            {
                bool too_close = false;
                for (const auto &existing_pos : positions)
                {
                    if ((existing_pos - pos).norm() < min_distance)
                    {
                        too_close = true;
                        break;
                    }
                }
                if (too_close)
                    continue;
            }

            // Accept this point
            positions.push_back(pos);

            // Compute orientation facing toward center
            Eigen::Vector3d direction = center - pos;
            direction.normalize();

            const auto quat_delta = geometry_utils::quatFromTwoVectors(cap_z, direction);
            const Eigen::Quaterniond q_delta = geometry_utils::arrayToEigenQuat(quat_delta);
            const Eigen::Quaterniond q_world = q_delta * base_quat;

            Viewpoint vp(pos, geometry_utils::eigenQuatToArray(q_world));
            vp.target = center;
            viewpoints.push_back(vp);
        }

        if (viewpoints.size() < static_cast<size_t>(num_samples))
        {
            std::cerr << "Warning: Only generated " << viewpoints.size() << "/" << num_samples
                      << " viewpoints. Try reducing min_distance or increasing the sampling region."
                      << std::endl;
        }

        return viewpoints;
    }

    double computeInformationGain(
        const Viewpoint &viewpoint,
        const std::shared_ptr<OctoMapInterface> &octomap_interface,
        double fov,
        int width,
        int height,
        double max_range,
        double resolution,
        int num_rays,
        bool use_bbox)
    {
        if (!octomap_interface || width <= 0 || height <= 0 || max_range <= 0 || resolution <= 0)
        {
            return 0.0;
        }

        // Check if tree is available
        if (!octomap_interface->isTreeAvailable())
        {
            std::cerr << "Octomap tree is not available" << std::endl;
            return 0.0;
        }

        // Get bounding box if needed
        octomap::point3d bbox_min, bbox_max;
        if (use_bbox)
        {
            if (!octomap_interface->getBoundingBox(bbox_min, bbox_max))
            {
                std::cerr << "Failed to get bounding box from octomap" << std::endl;
                return 0.0;
            }
        }

        // Get octree depth
        const int max_depth = octomap_interface->getOctreeDepth();
        if (max_depth == 0)
        {
            return 0.0;
        }

        // Compute ray strides
        const auto [stride_u, stride_v] = geometry_utils::computeRayStrides(width, height, num_rays);

        // Camera extrinsics
        const Eigen::Matrix3d R_wc = geometry_utils::quatToRotMat(viewpoint.orientation);

        // Camera intrinsics
        const double hfov = fov * M_PI / 180.0;
        const double focal = (width / 2.0) / std::tan(hfov / 2.0);
        const double cx = (width - 1) * 0.5;
        const double cy = (height - 1) * 0.5;

        int total_unknown = 0;
        int num_rays_cast = 0;

        for (int v = 0; v < height; v += stride_v)
        {
            for (int u = 0; u < width; u += stride_u)
            {
                // Ray in camera frame
                const double x = (u - cx) / focal;
                const double y = (v - cy) / focal;
                Eigen::Vector3d ray_cam(x, y, 1.0);
                ray_cam.normalize();

                // Transform to world frame
                Eigen::Vector3d ray_world = R_wc * ray_cam;
                ray_world.normalize();

                // March along the ray
                int unknown_count = 0;
                bool prev_point_in_bbox = false;

                const int num_steps = static_cast<int>(max_range / resolution);
                for (int i = 0; i < num_steps; ++i)
                {
                    const double t = i * resolution;
                    const Eigen::Vector3d point = viewpoint.position + t * ray_world;
                    const octomap::point3d oct_point(point.x(), point.y(), point.z());

                    // Check bounding box if enabled
                    bool current_in_bbox = true;
                    if (use_bbox)
                    {
                        current_in_bbox = (oct_point.x() >= bbox_min.x() && oct_point.x() <= bbox_max.x() &&
                                           oct_point.y() >= bbox_min.y() && oct_point.y() <= bbox_max.y() &&
                                           oct_point.z() >= bbox_min.z() && oct_point.z() <= bbox_max.z());

                        // End ray if exiting bbox
                        if (!current_in_bbox && prev_point_in_bbox)
                        {
                            break;
                        }
                        prev_point_in_bbox = current_in_bbox;
                    }

                    // Only check voxels inside bbox (or if bbox disabled)
                    if (current_in_bbox)
                    {
                        octomap::OcTreeNode *node = nullptr;
                        const bool found = octomap_interface->searchOctree(oct_point, node);

                        if (!found || node == nullptr)
                        {
                            // Unknown voxel
                            unknown_count++;
                        }
                        else if (octomap_interface->isVoxelOccupied(oct_point))
                        {
                            // Occupied voxel - stop ray
                            break;
                        }
                        // Free voxel - continue marching
                    }
                }

                total_unknown += unknown_count;
                num_rays_cast++;
            }
        }

        return num_rays_cast > 0 ? static_cast<double>(total_unknown) / num_rays_cast : 0.0;
    }

    double computeSemanticInformationGain(
        const Viewpoint &viewpoint,
        const std::shared_ptr<OctoMapInterface> &octomap_interface,
        double fov,
        int width,
        int height,
        double max_range,
        double resolution,
        int num_rays,
        bool use_bbox,
        double beta)
    {
        // Note: This is a simplified version that treats the semantic octree
        // like a regular octree. For full semantic functionality, you would
        // need to integrate with the SemanticOctoMap class and access the
        // semantic_map to get confidence values.

        // Suppress unused parameter warning
        (void)beta;

        // For now, this returns the same as computeInformationGain
        // You can extend this by passing a SemanticOctoMap pointer instead

        return computeInformationGain(
            viewpoint, octomap_interface, fov, width, height,
            max_range, resolution, num_rays, use_bbox);
    }

} // namespace husky_xarm6_mcr_nbv_planner
