"""
Next-Best-View (NBV) Planner using sampling-based approach.

Samples candidate viewpoints, evaluates their information gain,
and selects the best view based on volumetric improvement.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from typing import List, Tuple, Optional, Dict
import time

# Import from your modules
from ..perception.volumetric_map import VolumetricMap
from ..motion_interface.moveit_interface import MoveItInterface


class NextBestViewPlanner:
    """
    Sampling-based Next-Best-View planner.
    
    Strategy:
    1. Sample candidate viewpoints around target object/region
    2. Evaluate volumetric information gain for each candidate
    3. Check feasibility (reachability, collision-free)
    4. Select view with highest gain
    5. Execute motion to selected view
    """
    
    def __init__(
        self,
        node: Node,
        volumetric_map: VolumetricMap,
        moveit_interface: MoveItInterface,
        target_region: Tuple[Point, float] = None
    ):
        """
        Initialize NBV planner.
        
        Args:
            node: ROS2 node
            volumetric_map: Volumetric map for information gain
            moveit_interface: MoveIt interface for motion
            target_region: (center, radius) of target scanning region
        """
        self.node = node
        self.map = volumetric_map
        self.moveit = moveit_interface
        
        # Target region to scan (e.g., tree location)
        if target_region is None:
            # Default: 1m sphere at (1.5, 0, 1.0)
            center = Point(x=1.5, y=0.0, z=1.0)
            radius = 1.0
            target_region = (center, radius)
        
        self.target_center = target_region[0]
        self.target_radius = target_region[1]
        
        # Planning parameters
        self.num_samples = 50  # Number of candidate views to sample
        self.camera_distance = 1.5  # Distance from target center
        self.camera_fov = 60.0  # degrees
        self.camera_max_range = 3.0  # meters
        
        # Results
        self.best_view = None
        self.candidate_views = []
        
    def sample_candidate_views(self, num_samples: int = None) -> List[Pose]:
        """
        Sample candidate viewpoints around target region.
        
        Uses spherical sampling to generate views looking at target.
        
        Args:
            num_samples: Number of views to sample
            
        Returns:
            List of candidate poses
        """
        if num_samples is None:
            num_samples = self.num_samples
        
        candidates = []
        
        # Sample on a sphere around target
        # Using Fibonacci sphere for uniform distribution
        golden_ratio = (1 + np.sqrt(5)) / 2
        
        for i in range(num_samples):
            # Fibonacci sphere point
            theta = 2 * np.pi * i / golden_ratio
            phi = np.arccos(1 - 2 * (i + 0.5) / num_samples)
            
            # Only sample upper hemisphere (z > target center)
            if phi > np.pi / 2:
                phi = np.pi - phi
            
            # Convert to Cartesian
            x = self.camera_distance * np.sin(phi) * np.cos(theta)
            y = self.camera_distance * np.sin(phi) * np.sin(theta)
            z = self.camera_distance * np.cos(phi)
            
            # Position relative to target center
            pos = Point(
                x=self.target_center.x + x,
                y=self.target_center.y + y,
                z=self.target_center.z + z
            )
            
            # Orientation: look at target center
            orientation = self._look_at_orientation(
                pos, 
                self.target_center
            )
            
            # Create pose
            pose = Pose(position=pos, orientation=orientation)
            candidates.append(pose)
        
        self.node.get_logger().info(f'Sampled {len(candidates)} candidate views')
        self.candidate_views = candidates
        
        return candidates
    
    def _look_at_orientation(
        self, 
        eye: Point, 
        target: Point
    ) -> Quaternion:
        """
        Compute orientation quaternion to look from eye to target.
        
        Args:
            eye: Camera position
            target: Point to look at
            
        Returns:
            Quaternion orientation
        """
        # Direction vector
        direction = np.array([
            target.x - eye.x,
            target.y - eye.y,
            target.z - eye.z
        ])
        direction = direction / np.linalg.norm(direction)
        
        # Assume camera z-axis points forward
        # x-axis right, y-axis down (typical camera frame)
        
        # Up vector (world z-axis)
        up = np.array([0, 0, 1])
        
        # Right vector
        right = np.cross(up, direction)
        if np.linalg.norm(right) < 1e-6:
            # Direction is vertical, use different up
            up = np.array([1, 0, 0])
            right = np.cross(up, direction)
        right = right / np.linalg.norm(right)
        
        # Recalculate up
        up = np.cross(direction, right)
        
        # Build rotation matrix
        # Camera frame: x=right, y=down, z=forward
        rot_matrix = np.column_stack([right, -up, direction])
        
        # Convert to quaternion
        quat = self._rotation_matrix_to_quaternion(rot_matrix)
        
        return Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
    
    def _rotation_matrix_to_quaternion(self, R: np.ndarray) -> np.ndarray:
        """Convert 3x3 rotation matrix to quaternion [x, y, z, w]."""
        trace = np.trace(R)
        
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        
        return np.array([x, y, z, w])
    
    def evaluate_candidates(
        self, 
        candidates: List[Pose]
    ) -> List[Tuple[Pose, float, bool]]:
        """
        Evaluate all candidate views.
        
        Args:
            candidates: List of candidate poses
            
        Returns:
            List of (pose, information_gain, is_feasible) tuples
        """
        results = []
        
        camera_params = {
            'fov': self.camera_fov,
            'max_range': self.camera_max_range
        }
        
        for i, pose in enumerate(candidates):
            # Compute information gain
            gain = self.map.compute_information_gain(pose, camera_params)
            
            # Check feasibility (simplified - in practice check IK + collisions)
            is_feasible = self._check_feasibility(pose)
            
            results.append((pose, gain, is_feasible))
            
            if (i + 1) % 10 == 0:
                self.node.get_logger().info(
                    f'Evaluated {i + 1}/{len(candidates)} candidates'
                )
        
        return results
    
    def _check_feasibility(self, pose: Pose) -> bool:
        """
        Check if a pose is feasible (reachable and collision-free).
        
        Args:
            pose: Pose to check
            
        Returns:
            True if feasible
        """
        # Check workspace bounds
        min_bounds, max_bounds = self.moveit.get_workspace_bounds()
        
        pos = np.array([pose.position.x, pose.position.y, pose.position.z])
        
        if np.any(pos < min_bounds) or np.any(pos > max_bounds):
            return False
        
        # Would also check IK solvability and collisions
        # Placeholder: assume feasible if in workspace
        return True
    
    def select_best_view(
        self, 
        evaluated_views: List[Tuple[Pose, float, bool]]
    ) -> Optional[Pose]:
        """
        Select the best view from evaluated candidates.
        
        Args:
            evaluated_views: List of (pose, gain, feasible) tuples
            
        Returns:
            Best pose, or None if no feasible views
        """
        # Filter to feasible views
        feasible = [(p, g) for p, g, f in evaluated_views if f]
        
        if not feasible:
            self.node.get_logger().warn('No feasible views found!')
            return None
        
        # Select view with maximum gain
        best_pose, best_gain = max(feasible, key=lambda x: x[1])
        
        self.node.get_logger().info(
            f'Best view selected with gain: {best_gain:.2f}'
        )
        
        self.best_view = best_pose
        return best_pose
    
    def plan_iteration(self) -> Optional[Pose]:
        """
        Execute one iteration of NBV planning.
        
        Returns:
            Selected next-best view, or None if planning failed
        """
        self.node.get_logger().info('=== Starting NBV planning iteration ===')
        
        # 1. Sample candidates
        candidates = self.sample_candidate_views()
        
        # 2. Evaluate candidates
        evaluated = self.evaluate_candidates(candidates)
        
        # 3. Select best
        best_view = self.select_best_view(evaluated)
        
        if best_view is None:
            return None
        
        # 4. Plan motion (optional - can be done separately)
        # self.moveit.plan_to_pose(best_view)
        
        return best_view
    
    def get_exploration_progress(self) -> Dict:
        """
        Get current exploration progress metrics.
        
        Returns:
            Dictionary with progress information
        """
        unknown, free, occupied = self.map.get_occupancy_ratio()
        
        return {
            'unknown_ratio': unknown,
            'free_ratio': free,
            'occupied_ratio': occupied,
            'explored_ratio': 1.0 - unknown
        }


class NBVPlannerNode(Node):
    """ROS2 node for Next-Best-View planning."""
    
    def __init__(self):
        super().__init__('nbv_planner_node')
        
        # Parameters
        self.declare_parameter('num_samples', 50)
        self.declare_parameter('camera_distance', 1.5)
        self.declare_parameter('target_x', 1.5)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_z', 1.0)
        self.declare_parameter('target_radius', 1.0)
        
        # Initialize components
        self.volumetric_map = VolumetricMap(
            resolution=0.05,
            map_size=(10.0, 10.0, 5.0)
        )
        
        self.moveit_interface = MoveItInterface(self)
        
        # Set target region
        target_center = Point(
            x=self.get_parameter('target_x').value,
            y=self.get_parameter('target_y').value,
            z=self.get_parameter('target_z').value
        )
        target_radius = self.get_parameter('target_radius').value
        
        self.planner = NextBestViewPlanner(
            node=self,
            volumetric_map=self.volumetric_map,
            moveit_interface=self.moveit_interface,
            target_region=(target_center, target_radius)
        )
        
        # Visualization publisher
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/nbv_candidates',
            10
        )
        
        # Timer for periodic planning
        self.plan_timer = self.create_timer(10.0, self.planning_callback)
        
        self.get_logger().info('NBV Planner initialized')
    
    def planning_callback(self):
        """Periodic planning callback."""
        self.get_logger().info('Running NBV planning...')
        
        best_view = self.planner.plan_iteration()
        
        if best_view:
            # Visualize candidates
            self.visualize_candidates()
            
            # Get progress
            progress = self.planner.get_exploration_progress()
            self.get_logger().info(
                f'Exploration progress: {progress["explored_ratio"]:.1%}'
            )
    
    def visualize_candidates(self):
        """Publish visualization markers for candidate views."""
        marker_array = MarkerArray()
        
        for i, pose in enumerate(self.planner.candidate_views):
            marker = Marker()
            marker.header.frame_id = "a200_base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "nbv_candidates"
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = pose
            marker.scale.x = 0.3
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.a = 0.5
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            
            marker_array.markers.append(marker)
        
        # Highlight best view
        if self.planner.best_view:
            marker = Marker()
            marker.header.frame_id = "a200_base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "nbv_best"
            marker.id = 9999
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = self.planner.best_view
            marker.scale.x = 0.5
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            
            marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = NBVPlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
