"""
Volumetric mapping for next-best-view planning with RViz visualization.

Changes vs. the original:
- Correct point-cloud accumulation (Open3D) and optional downsampling
- Maintains sets of OCCUPIED and FREE voxels (FREE from ray carving with DDA)
- Publishes RViz markers (CUBE_LIST) for occupied voxels
- Publishes an optional accumulated PointCloud2 for debugging
- Uses TF2 to transform PointCloud2 into map_frame
- Information gain via true pinhole frustum rays from camera intrinsics
"""

from __future__ import annotations
import math
import numpy as np
from typing import List, Tuple, Optional, Set

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import open3d as o3d

from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2

from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration

import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration as RclpyDuration
from rclpy.time import Time as RclpyTime

# ----------------------------
# Math utilities
# ----------------------------

def quaternion_to_rot_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Convert quaternion (x,y,z,w) to 3x3 rotation matrix."""
    # Normalize to be safe
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm == 0.0:
        return np.eye(3)
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    R = np.array([
        [1.0 - 2.0*(yy + zz),     2.0*(xy - wz),         2.0*(xz + wy)],
        [    2.0*(xy + wz),   1.0 - 2.0*(xx + zz),       2.0*(yz - wx)],
        [    2.0*(xz - wy),       2.0*(yz + wx),     1.0 - 2.0*(xx + yy)],
    ], dtype=np.float64)
    return R

def make_transform_matrix(t: TransformStamped) -> np.ndarray:
    """Build 4x4 homogeneous transform from TransformStamped."""
    tx = t.transform.translation.x
    ty = t.transform.translation.y
    tz = t.transform.translation.z
    qx = t.transform.rotation.x
    qy = t.transform.rotation.y
    qz = t.transform.rotation.z
    qw = t.transform.rotation.w
    R = quaternion_to_rot_matrix(qx, qy, qz, qw)
    T = np.eye(4, dtype=np.float64)
    T[:3,:3] = R
    T[:3, 3] = np.array([tx, ty, tz], dtype=np.float64)
    return T

def transform_points(T: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """Apply 4x4 transform to Nx3 points."""
    if pts.size == 0:
        return pts
    ones = np.ones((pts.shape[0], 1), dtype=np.float64)
    hom = np.hstack([pts, ones])
    out = (T @ hom.T).T
    return out[:, :3]

# ----------------------------
# Volumetric map
# ----------------------------

class VolumetricMap:
    def __init__(
        self,
        resolution: float = 0.05,  # voxel edge in meters
        map_size: Tuple[float, float, float] = (10.0, 10.0, 5.0),
        origin: Tuple[float, float, float] = (-5.0, -5.0, 0.0),
        max_depth: int = 8,
        downsample_voxel: Optional[float] = None,
        rebuild_octree_every_n: int = 1,
    ):
        self.resolution = float(resolution)
        self.map_size = tuple(map(float, map_size))
        self.origin = np.array(origin, dtype=np.float64)
        self.max_depth = int(max_depth)
        self.downsample_voxel = float(downsample_voxel) if downsample_voxel else None
        self.rebuild_octree_every_n = max(1, int(rebuild_octree_every_n))

        # Derived grid dims
        self.grid_dims = np.array(
            [int(s / self.resolution) for s in self.map_size], dtype=np.int32
        )

        # Open3D point-cloud and (optional) octree
        self.pcd = o3d.geometry.PointCloud()
        self.octree: Optional[o3d.geometry.Octree] = None

        # Voxel occupancy sets
        self.occupied_voxels: Set[Tuple[int, int, int]] = set()
        self.free_voxels: Set[Tuple[int, int, int]] = set()

        # Stats
        self.total_points_observed = 0
        self._update_counter = 0

        print(f"[VolumetricMap] resolution={self.resolution} m, "
              f"map_size={self.map_size} m, origin={self.origin.tolist()}, "
              f"grid={self.grid_dims.tolist()}, max_depth={self.max_depth}")

    # ---------- Voxel helpers ----------

    def _point_to_voxel(self, p: np.ndarray) -> Optional[Tuple[int, int, int]]:
        """World point -> voxel index (ix,iy,iz), or None if outside bounds."""
        g = (p - self.origin) / self.resolution
        ix, iy, iz = int(math.floor(g[0])), int(math.floor(g[1])), int(math.floor(g[2]))
        if (0 <= ix < self.grid_dims[0] and
            0 <= iy < self.grid_dims[1] and
            0 <= iz < self.grid_dims[2]):
            return (ix, iy, iz)
        return None

    def _voxel_center(self, key: Tuple[int,int,int]) -> np.ndarray:
        """Voxel center world coordinates."""
        v = np.array(key, dtype=np.float64)
        return self.origin + (v + 0.5) * self.resolution

    # ---------- Ray carving (3D DDA) ----------

    def _raycarve_voxels(self, start_pt: np.ndarray, end_pt: np.ndarray):
        """
        Mark voxels along the line segment [start_pt -> end_pt]:
        - Intermediate traversed voxels as FREE
        - Hit/end voxel as OCCUPIED (if inside map)
        """
        start_key = self._point_to_voxel(start_pt)
        end_key   = self._point_to_voxel(end_pt)
        # If the end is outside, we still carve free along the in-bounds portion
        # so we clamp traversal to the map bounds via incremental stepping.

        # DDA setup in voxel coordinates
        def world_to_voxel_coord(p):
            return (p - self.origin) / self.resolution

        v0 = world_to_voxel_coord(start_pt)
        v1 = world_to_voxel_coord(end_pt)

        # Direction and step
        dv = v1 - v0
        length = np.linalg.norm(dv)
        if length < 1e-8:
            # Degenerate ray
            if start_key is not None:
                self.free_voxels.add(start_key)
            return

        step = np.sign(dv).astype(int)
        t_max = np.zeros(3, dtype=np.float64)
        t_delta = np.zeros(3, dtype=np.float64)

        # Current voxel
        v = np.floor(v0).astype(int)

        # Initialize t_max / t_delta per-axis
        for i in range(3):
            if step[i] != 0:
                # Distance to next boundary
                next_boundary = (v[i] + (1 if step[i] > 0 else 0))
                t_max[i] = (next_boundary - v0[i]) / dv[i]
                t_delta[i] = 1.0 / abs(dv[i])
            else:
                t_max[i] = np.inf
                t_delta[i] = np.inf

        # Traverse until we reach end voxel or leave bounds or exceed ray length
        # Work in "t" parameter from 0 -> 1 along v0 + t*dv
        t = 0.0
        max_t = 1.0 + 1e-6

        # Helper: bounds check
        def in_bounds(vi):
            return (0 <= vi[0] < self.grid_dims[0] and
                    0 <= vi[1] < self.grid_dims[1] and
                    0 <= vi[2] < self.grid_dims[2])

        # Mark traversed as FREE, last one as OCCUPIED (if hit point is inside)
        while in_bounds(v) and t <= max_t:
            key = (int(v[0]), int(v[1]), int(v[2]))
            # Decide next axis to step
            axis = int(np.argmin(t_max))
            t = t_max[axis]

            # If we are at the end voxel, stop after marking occupied below
            at_end = (end_key is not None and key == end_key)

            if not at_end:
                self.free_voxels.add(key)

            # Step to next voxel
            v[axis] += step[axis]
            t_max[axis] += t_delta[axis]

            if at_end:
                # Mark end voxel occupied
                self.occupied_voxels.add(key)
                break

        # If end voxel is in-bounds but loop ended before marking it
        if end_key is not None and end_key not in self.occupied_voxels:
            self.occupied_voxels.add(end_key)

        # Clear any conflicts: occupied overrides free
        # (Remove any voxel that appears in both)
        if self.occupied_voxels & self.free_voxels:
            self.free_voxels.difference_update(self.occupied_voxels)

    # ---------- Map update ----------

    def update_from_pointcloud(
        self,
        points_world: np.ndarray,
        sensor_origin_world: np.ndarray,
        do_downsample: bool = True
    ) -> Tuple[int, int]:
        """
        Update the map with a world-frame point cloud.
        Ray-casts from sensor origin to each point to carve FREE and OCCUPIED.

        Returns:
            (new_occupied_voxels, new_free_voxels)
        """
        if points_world.size == 0:
            return (0, 0)

        # Optionally downsample for speed
        pts = points_world
        if do_downsample and self.downsample_voxel and pts.shape[0] > 0:
            tmp_pcd = o3d.geometry.PointCloud()
            tmp_pcd.points = o3d.utility.Vector3dVector(pts)
            tmp_pcd = tmp_pcd.voxel_down_sample(self.downsample_voxel)
            pts = np.asarray(tmp_pcd.points, dtype=np.float64)

        # Accumulate into O3D point cloud
        new_pcd = o3d.geometry.PointCloud()
        new_pcd.points = o3d.utility.Vector3dVector(pts)
        self.pcd += new_pcd  # correct way to merge

        # Ray carve per point (fast DDA)
        before_occ = len(self.occupied_voxels)
        before_free = len(self.free_voxels)

        for p in pts:
            self._raycarve_voxels(sensor_origin_world, p)

        # Rebuild octree occasionally (optional)
        self._update_counter += 1
        if self._update_counter % self.rebuild_octree_every_n == 0:
            self.octree = o3d.geometry.Octree(max_depth=self.max_depth)
            self.octree.convert_from_point_cloud(self.pcd, size_expand=0.01)

        self.total_points_observed += pts.shape[0]

        return (len(self.occupied_voxels) - before_occ, len(self.free_voxels) - before_free)

    # ---------- Info gain via frustum rays ----------

    def compute_information_gain(
        self,
        candidate_pose: Pose,
        camera_params: dict
    ) -> float:
        """
        Cast rays through a decimated image grid from the candidate camera pose.
        For each ray, count a gain of +1 if the ray encounters an UNKNOWN voxel
        before hitting an OCCUPIED voxel or max range.
        """
        # Camera intrinsics
        fx = float(camera_params.get('fx', 500.0))
        fy = float(camera_params.get('fy', 500.0))
        cx = float(camera_params.get('cx', 320.0))
        cy = float(camera_params.get('cy', 240.0))
        width  = int(camera_params.get('width', 640))
        height = int(camera_params.get('height', 480))
        max_range = float(camera_params.get('max_range', 3.0))
        pixel_stride = int(camera_params.get('pixel_stride', 16))  # decimation for speed

        # Pose -> origin + orientation
        cam_t = np.array([candidate_pose.position.x,
                          candidate_pose.position.y,
                          candidate_pose.position.z], dtype=np.float64)
        R = quaternion_to_rot_matrix(candidate_pose.orientation.x,
                                     candidate_pose.orientation.y,
                                     candidate_pose.orientation.z,
                                     candidate_pose.orientation.w)

        # For each decimated pixel, form a ray in camera frame and rotate to world
        gain = 0.0
        for v in range(0, height, pixel_stride):
            for u in range(0, width, pixel_stride):
                # Direction in camera frame (pinhole, z-forward)
                x = (u - cx) / fx
                y = (v - cy) / fy
                dir_cam = np.array([x, y, 1.0], dtype=np.float64)
                dir_cam /= np.linalg.norm(dir_cam) + 1e-12

                # World ray
                dir_world = (R @ dir_cam)
                end_world = cam_t + dir_world * max_range

                # March in voxel space to see if first touch is UNKNOWN
                hit_unknown = self._first_unknown_along_ray(cam_t, end_world)
                if hit_unknown:
                    gain += 1.0
        return gain

    def _first_unknown_along_ray(self, start_pt: np.ndarray, end_pt: np.ndarray) -> bool:
        """
        Traverse voxels along ray; return True if the first non-free voxel
        encountered is UNKNOWN (i.e., not in free or occupied).
        """
        def in_bounds(v):
            return (0 <= v[0] < self.grid_dims[0] and
                    0 <= v[1] < self.grid_dims[1] and
                    0 <= v[2] < self.grid_dims[2])

        # DDA like above, but classification only
        v0 = (start_pt - self.origin) / self.resolution
        v1 = (end_pt   - self.origin) / self.resolution
        dv = v1 - v0
        length = np.linalg.norm(dv)
        if length < 1e-8:
            key0 = self._point_to_voxel(start_pt)
            return (key0 is not None and
                    key0 not in self.free_voxels and
                    key0 not in self.occupied_voxels)

        step = np.sign(dv).astype(int)
        v = np.floor(v0).astype(int)
        t_max = np.zeros(3, dtype=np.float64)
        t_delta = np.zeros(3, dtype=np.float64)
        for i in range(3):
            if step[i] != 0:
                next_boundary = (v[i] + (1 if step[i] > 0 else 0))
                t_max[i] = (next_boundary - v0[i]) / dv[i]
                t_delta[i] = 1.0 / abs(dv[i])
            else:
                t_max[i] = np.inf
                t_delta[i] = np.inf

        t, max_t = 0.0, 1.0 + 1e-6
        while in_bounds(v) and t <= max_t:
            key = (int(v[0]), int(v[1]), int(v[2]))
            # If we've reached an OCCUPIED voxel first -> not unknown
            if key in self.occupied_voxels:
                return False
            # If we hit a voxel that's neither free nor occupied -> unknown first
            if key not in self.free_voxels:
                return True
            # Step
            axis = int(np.argmin(t_max))
            t = t_max[axis]
            v[axis] += step[axis]
            t_max[axis] += t_delta[axis]
        # If the entire ray was free or out-of-bounds: no unknown first
        return False

    # ---------- Map stats & access ----------

    def get_occupancy_ratio(self) -> Tuple[float, float, float]:
        """Return (unknown_ratio, free_ratio, occupied_ratio)."""
        total_voxels = int(self.grid_dims[0] * self.grid_dims[1] * self.grid_dims[2])
        occupied = len(self.occupied_voxels)
        free = len(self.free_voxels)
        # Clamp in case of overlap logic changes
        if occupied + free > total_voxels:
            free = max(0, total_voxels - occupied)
        unknown = max(0, total_voxels - occupied - free)
        if total_voxels == 0:
            return (0.0, 0.0, 0.0)
        return (unknown / total_voxels, free / total_voxels, occupied / total_voxels)

    def get_point_cloud(self) -> o3d.geometry.PointCloud:
        return self.pcd

    def get_octree(self) -> Optional[o3d.geometry.Octree]:
        return self.octree

    def save_point_cloud(self, filename: str):
        if len(self.pcd.points) > 0:
            o3d.io.write_point_cloud(filename, self.pcd)
            print(f"Saved point cloud with {len(self.pcd.points)} points to {filename}")
        else:
            print("No points to save")

# ----------------------------
# ROS 2 Node
# ----------------------------

class VolumetricMapNode(Node):
    """ROS2 node for volumetric mapping with RViz outputs and TF2 integration."""

    def __init__(self):
        super().__init__('volumetric_map_node')

        # Note: use_sim_time is automatically declared by ROS2 and set via launch file
        
        # Parameters
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('map_size_x', 10.0)
        self.declare_parameter('map_size_y', 10.0)
        self.declare_parameter('map_size_z', 5.0)
        self.declare_parameter('origin_x', -5.0)
        self.declare_parameter('origin_y', -5.0)
        self.declare_parameter('origin_z', 0.0)
        self.declare_parameter('max_depth', 8)
        self.declare_parameter('pointcloud_topic', '/firefly_left/points2')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('downsample_voxel', 0.05)   # for incoming clouds
        self.declare_parameter('rebuild_octree_every_n', 5)
        self.declare_parameter('publish_accumulated_cloud', True)

        # Camera intrinsics for frustum IG (can be your Firefly intrinsics)
        self.declare_parameter('camera.fx', 500.0)
        self.declare_parameter('camera.fy', 500.0)
        self.declare_parameter('camera.cx', 320.0)
        self.declare_parameter('camera.cy', 240.0)
        self.declare_parameter('camera.width', 640)
        self.declare_parameter('camera.height', 480)
        self.declare_parameter('camera.max_range', 3.0)
        self.declare_parameter('camera.pixel_stride', 16)

        # Read parameters
        resolution = float(self.get_parameter('resolution').value)
        map_size = (
            float(self.get_parameter('map_size_x').value),
            float(self.get_parameter('map_size_y').value),
            float(self.get_parameter('map_size_z').value),
        )
        origin = (
            float(self.get_parameter('origin_x').value),
            float(self.get_parameter('origin_y').value),
            float(self.get_parameter('origin_z').value),
        )
        max_depth = int(self.get_parameter('max_depth').value)
        pc_topic = self.get_parameter('pointcloud_topic').value
        self.map_frame: str = self.get_parameter('map_frame').value
        downsample_voxel = float(self.get_parameter('downsample_voxel').value)
        rebuild_every = int(self.get_parameter('rebuild_octree_every_n').value)
        self.publish_accumulated_cloud: bool = bool(self.get_parameter('publish_accumulated_cloud').value)

        # Camera params dict
        self.camera_params = dict(
            fx=float(self.get_parameter('camera.fx').value),
            fy=float(self.get_parameter('camera.fy').value),
            cx=float(self.get_parameter('camera.cx').value),
            cy=float(self.get_parameter('camera.cy').value),
            width=int(self.get_parameter('camera.width').value),
            height=int(self.get_parameter('camera.height').value),
            max_range=float(self.get_parameter('camera.max_range').value),
            pixel_stride=int(self.get_parameter('camera.pixel_stride').value),
        )

        # Volumetric map
        self.map = VolumetricMap(
            resolution=resolution,
            map_size=map_size,
            origin=origin,
            max_depth=max_depth,
            downsample_voxel=downsample_voxel if downsample_voxel > 0.0 else None,
            rebuild_octree_every_n=rebuild_every
        )

        # TF2 - use smaller cache for real-time operation
        self.tf_buffer = tf2_ros.Buffer(cache_time=RclpyDuration(seconds=3.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribers - use depth=1 to only process latest messages
        qos_best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1  # Only keep latest message to avoid processing stale data
        )

        self.pc_sub = self.create_subscription(
            PointCloud2,
            pc_topic,
            self.pointcloud_callback,
            qos_profile=qos_best_effort
        )

        # Publishers
        self.marker_pub = self.create_publisher(Marker, 'volumetric_map/occupied_voxels', 1)
        self.pcd_pub = self.create_publisher(PointCloud2, 'volumetric_map/points_accumulated', 1)

        self.update_count = 0

        self.get_logger().info(
            f'Volumetric map initialized. size={map_size} origin={origin} res={resolution} '
            f'max_depth={max_depth} pc_topic={pc_topic} frame={self.map_frame}'
        )

    # ---------- Callbacks ----------

    def pointcloud_callback(self, msg: PointCloud2):
        """Process incoming PointCloud2 to update volumetric map."""
        # 1) Convert incoming PointCloud2 to Nx3 in the cloud frame
        pts = []
        for p in pc2.read_points(msg, skip_nans=True, field_names=("x","y","z")):
            pts.append([p[0], p[1], p[2]])
        if not pts:
            return
        pts = np.asarray(pts, dtype=np.float64)

        cloud_frame = msg.header.frame_id
        # Use Time(0) to get the latest available transform instead of exact message time
        # This is more robust in simulation where timing can be tricky

        # 2) Transform points & sensor origin into map_frame
        sensor_origin_world = np.zeros(3, dtype=np.float64)
        try:
            # Use rclpy.time.Time() for "latest available" transform
            T = self.tf_buffer.lookup_transform(
                self.map_frame, 
                cloud_frame, 
                rclpy.time.Time(),  # Latest available transform
                timeout=RclpyDuration(seconds=0.1)
            )
            Tm = make_transform_matrix(T)
            pts_world = transform_points(Tm, pts)
            sensor_origin_world = Tm[:3, 3]
            
            if self.update_count % 10 == 0:
                self.get_logger().info(
                    f"Transform OK: {cloud_frame} -> {self.map_frame}, "
                    f"sensor origin: [{sensor_origin_world[0]:.2f}, {sensor_origin_world[1]:.2f}, {sensor_origin_world[2]:.2f}]"
                )
        except TransformException as ex:
            # CRITICAL: Skip this message if transform fails - don't use incorrect fallback
            self.get_logger().warn(
                f"TF lookup failed from {cloud_frame} -> {self.map_frame}: {ex}. "
                "SKIPPING this point cloud message.",
                throttle_duration_sec=5.0  # Only warn once per 5 seconds
            )
            return  # Skip processing this message

        # 3) Update volumetric map with ray carving
        new_occ, new_free = self.map.update_from_pointcloud(pts_world, sensor_origin_world, do_downsample=True)
        self.update_count += 1

        # 4) Log ratios & publish visualization
        if new_occ > 0 or new_free > 0 or (self.update_count % 10 == 0):
            unknown, free, occ = self.map.get_occupancy_ratio()
            self.get_logger().info(
                f'Update #{self.update_count}: +occ={new_occ}, +free={new_free}, '
                f'total_occ={len(self.map.occupied_voxels)}, '
                f'Unknown={unknown:.2%}, Free={free:.2%}, Occupied={occ:.2%}'
            )

        self.publish_occupied_voxels_marker()

        if self.publish_accumulated_cloud:
            self.publish_accumulated_pointcloud()

    # ---------- Publishers ----------

    def publish_occupied_voxels_marker(self):
        marker = Marker()
        marker.header = Header(frame_id=self.map_frame, stamp=self.get_clock().now().to_msg())
        marker.ns = "volumetric_map"
        marker.id = 1
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.scale.x = float(self.map.resolution)
        marker.scale.y = float(self.map.resolution)
        marker.scale.z = float(self.map.resolution)
        marker.color = ColorRGBA(r=0.1, g=0.4, b=1.0, a=0.35)  # semi-transparent blue
        marker.lifetime = Duration(sec=0, nanosec=0)  # persist until next update

        # Populate cube centers
        if self.map.occupied_voxels:
            centers = [self.map._voxel_center(k) for k in self.map.occupied_voxels]
            marker.points = [Point(x=float(c[0]), y=float(c[1]), z=float(c[2])) for c in centers]
        else:
            marker.points = []

        self.marker_pub.publish(marker)

    def publish_accumulated_pointcloud(self):
        if len(self.map.pcd.points) == 0:
            return
        pts = np.asarray(self.map.pcd.points, dtype=np.float32)
        header = Header(frame_id=self.map_frame, stamp=self.get_clock().now().to_msg())
        cloud = pc2.create_cloud_xyz32(header, pts.tolist())
        self.pcd_pub.publish(cloud)

    # ---------- Optional: public API for planners ----------

    def compute_information_gain(self, candidate_pose: Pose) -> float:
        """Expose frustum IG using current camera params."""
        return self.map.compute_information_gain(candidate_pose, self.camera_params)

# ----------------------------
# Entrypoint
# ----------------------------

def main(args=None):
    rclpy.init(args=args)
    node = VolumetricMapNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
