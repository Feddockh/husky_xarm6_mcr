"""
Robot Localization Launch File

Starts sensor fusion for GPS + IMU + Odometry using robot_localization package.
This corrects odometry drift and provides accurate map->odom->base_link transforms.

Components:
1. EKF Filter Node (map frame) - Fuses all sensors
2. Navsat Transform Node - Converts GPS lat/lon to map coordinates

Usage:
    ros2 launch husky_xarm6_mcr_bringup localization.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('husky_xarm6_mcr_bringup'),
            'config',
            'robot_localization_params.yaml'
        ]),
        description='Path to robot_localization configuration file'
    )
    
    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')
    
    # EKF Node for Map Frame (Global Localization with GPS)
    ekf_map_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('odometry/filtered', '/odometry/global')
        ]
    )
    
    # Navsat Transform Node - Converts GPS to Odometry
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('imu/data', '/imu/data'),
            ('gps/fix', '/gps/fix'),
            ('odometry/filtered', '/odometry/global'),
            ('odometry/gps', '/odometry/gps')
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        config_file_arg,
        ekf_map_node,
        navsat_transform_node,
    ])
