"""
MoveIt interface for motion planning and execution.

Provides a clean Python interface to MoveIt's move_group for:
- Planning to poses
- Executing trajectories
- Sampling IK solutions
- Collision checking
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    RobotState,
    MoveItErrorCodes
)
from moveit_msgs.action import MoveGroup
from sensor_msgs.msg import JointState
from typing import List, Optional, Tuple
import numpy as np


class MoveItInterface:
    """
    Simplified interface to MoveIt for NBV planning.
    
    Handles planning, execution, and IK queries without requiring
    the user to manage MoveIt details.
    """
    
    def __init__(
        self, 
        node: Node,
        group_name: str = "xarm6_manipulator",
        planning_frame: str = "a200_base_link",
        end_effector_link: str = "xarm6_link_tcp"
    ):
        """
        Initialize MoveIt interface.
        
        Args:
            node: ROS2 node for communication
            group_name: MoveIt planning group name
            planning_frame: Reference frame for planning
            end_effector_link: End effector link name
        """
        self.node = node
        self.group_name = group_name
        self.planning_frame = planning_frame
        self.end_effector_link = end_effector_link
        
        # Action client for MoveGroup
        self._move_group_client = ActionClient(
            node,
            MoveGroup,
            'move_action'
        )
        
        # Wait for action server
        self.node.get_logger().info('Waiting for MoveGroup action server...')
        self._move_group_client.wait_for_server()
        self.node.get_logger().info('MoveGroup action server connected')
        
        # Store current state
        self._current_joint_state = None
        
        # Subscribe to joint states
        self._joint_state_sub = node.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )
    
    def _joint_state_callback(self, msg: JointState):
        """Store current joint state."""
        self._current_joint_state = msg
    
    def plan_to_pose(
        self, 
        target_pose: Pose,
        timeout: float = 5.0,
        check_collision: bool = True
    ) -> Optional[dict]:
        """
        Plan to a target end-effector pose.
        
        Args:
            target_pose: Target pose for end effector
            timeout: Planning timeout in seconds
            check_collision: Whether to check collisions
            
        Returns:
            Planning result dict with trajectory, or None if failed
        """
        # Create planning request
        goal_msg = MoveGroup.Goal()
        
        # Set up request
        request = MotionPlanRequest()
        request.group_name = self.group_name
        request.num_planning_attempts = 5
        request.allowed_planning_time = timeout
        request.max_velocity_scaling_factor = 0.5
        request.max_acceleration_scaling_factor = 0.5
        
        # Set workspace bounds (adjust for your robot)
        request.workspace_parameters.header.frame_id = self.planning_frame
        request.workspace_parameters.min_corner.x = -2.0
        request.workspace_parameters.min_corner.y = -2.0
        request.workspace_parameters.min_corner.z = 0.0
        request.workspace_parameters.max_corner.x = 2.0
        request.workspace_parameters.max_corner.y = 2.0
        request.workspace_parameters.max_corner.z = 2.0
        
        # Set goal constraints
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.planning_frame
        pose_stamped.pose = target_pose
        
        # Create constraint
        constraints = Constraints()
        
        # Position constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = self.planning_frame
        pos_constraint.link_name = self.end_effector_link
        pos_constraint.target_point_offset.x = 0.0
        pos_constraint.target_point_offset.y = 0.0
        pos_constraint.target_point_offset.z = 0.0
        pos_constraint.weight = 1.0
        
        # Orientation constraint
        orient_constraint = OrientationConstraint()
        orient_constraint.header.frame_id = self.planning_frame
        orient_constraint.link_name = self.end_effector_link
        orient_constraint.orientation = target_pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 0.1
        orient_constraint.absolute_y_axis_tolerance = 0.1
        orient_constraint.absolute_z_axis_tolerance = 0.1
        orient_constraint.weight = 1.0
        
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(orient_constraint)
        
        request.goal_constraints.append(constraints)
        
        # Set planning options
        goal_msg.request = request
        goal_msg.planning_options.plan_only = False  # Plan and execute
        
        # Send goal
        self.node.get_logger().info(f'Planning to pose: {target_pose.position}')
        
        future = self._move_group_client.send_goal_async(goal_msg)
        
        # This is simplified - in practice you'd use proper async handling
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().warn('Goal rejected')
            return None
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=timeout)
        
        result = result_future.result()
        
        if result and result.result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.node.get_logger().info('Planning succeeded')
            return {'trajectory': result.result.planned_trajectory}
        else:
            error_code = result.result.error_code.val if result else -1
            self.node.get_logger().warn(f'Planning failed with code: {error_code}')
            return None
    
    def sample_ik_solutions(
        self, 
        target_pose: Pose,
        num_samples: int = 10
    ) -> List[List[float]]:
        """
        Sample multiple IK solutions for a target pose.
        
        This is useful for NBV planning to evaluate different
        joint configurations that achieve the same end-effector pose.
        
        Args:
            target_pose: Target end-effector pose
            num_samples: Number of solutions to sample
            
        Returns:
            List of joint configurations
        """
        # This would use MoveIt's IK service or compute IK module
        # Simplified implementation - you'd call the actual IK solver
        
        solutions = []
        
        # Placeholder: In practice, call IK solver multiple times
        # with different seed states to get diverse solutions
        
        self.node.get_logger().info(
            f'IK sampling not yet implemented (would sample {num_samples} solutions)'
        )
        
        return solutions
    
    def check_collision(
        self, 
        joint_state: Optional[List[float]] = None
    ) -> bool:
        """
        Check if a joint configuration is in collision.
        
        Args:
            joint_state: Joint configuration to check (None = current state)
            
        Returns:
            True if in collision, False otherwise
        """
        # Would use MoveIt's planning scene to check collisions
        # Placeholder implementation
        return False
    
    def get_current_pose(self) -> Optional[Pose]:
        """
        Get current end-effector pose.
        
        Returns:
            Current pose, or None if not available
        """
        # Would use TF2 to get transform from base to end effector
        # Placeholder implementation
        return None
    
    def get_workspace_bounds(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get reachable workspace bounds for the manipulator.
        
        Returns:
            (min_bounds, max_bounds) as 3D numpy arrays
        """
        # Approximate workspace based on robot specs
        # For xArm6: roughly 700mm reach
        min_bounds = np.array([-0.7, -0.7, 0.0])
        max_bounds = np.array([0.7, 0.7, 1.2])
        
        return min_bounds, max_bounds
    
    def stop_motion(self):
        """Emergency stop current motion."""
        # Would cancel current goal
        self.node.get_logger().warn('Stopping motion')
        # self._move_group_client.cancel_goal()


class MoveItInterfaceNode(Node):
    """Standalone node for testing MoveIt interface."""
    
    def __init__(self):
        super().__init__('moveit_interface_node')
        
        self.interface = MoveItInterface(self)
        
        self.get_logger().info('MoveIt interface ready')
    

def main(args=None):
    rclpy.init(args=args)
    node = MoveItInterfaceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
