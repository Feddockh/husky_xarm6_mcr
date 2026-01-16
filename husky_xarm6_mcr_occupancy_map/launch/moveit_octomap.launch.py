"""
Launch file for MoveIt-integrated occupancy map server
This publishes octomap_msgs/Octomap to /octomap_binary for MoveIt planning scene
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.02',
        description='Octomap resolution in meters'
    )
    
    max_range_arg = DeclareLaunchArgument(
        'max_range',
        default_value='3.0',
        description='Maximum sensor range in meters'
    )

    # MoveIt octomap server node with inline parameters
    moveit_octomap_server_node = Node(
        package='husky_xarm6_mcr_occupancy_map',
        executable='moveit_octomap_server',
        name='moveit_octomap_server',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            
            # Octomap parameters
            'resolution': LaunchConfiguration('resolution'),
            'map_frame': 'map',
            
            # Sensor model
            'max_range': LaunchConfiguration('max_range'),
            'min_range': 0.01,
            'prob_hit': 0.7,
            'prob_miss': 0.4,
            'clamp_min': 0.12,
            'clamp_max': 0.97,
            'occupancy_threshold': 0.5,
            
            # Ground filtering
            'filter_ground_plane': True,
            'ground_distance_threshold': 0.04,
            
            # Bounding box (limits octomap updates to this region)
            'use_bounding_box': True,
            'bbx_min_x': -1.0,
            'bbx_min_y': -2.0,
            'bbx_min_z': 0.0,
            'bbx_max_x': 1.0,
            'bbx_max_y': -0.5,
            'bbx_max_z': 2.0,
            
            # Topics
            'pointcloud_topic': '/firefly_left/points2',
            'planning_scene_world_topic': '/planning_scene_world',
            
            # Publishing
            'publish_free_voxels': True,
            'enable_visualization': True,
            'visualization_topic': 'occupancy_map_markers',
            'visualization_rate': 1.0,
            'octomap_publish_rate': 1.0,
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        resolution_arg,
        max_range_arg,
        moveit_octomap_server_node
    ])
