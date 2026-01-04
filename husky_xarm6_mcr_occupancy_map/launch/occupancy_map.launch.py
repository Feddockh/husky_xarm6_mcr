"""
Launch file for standalone occupancy map server
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('husky_xarm6_mcr_occupancy_map'),
            'config',
            'occupancy_map_params.yaml'
        ]),
        description='Path to occupancy map configuration file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Occupancy map server node
    occupancy_map_server_node = Node(
        package='husky_xarm6_mcr_occupancy_map',
        executable='occupancy_map_server',
        name='occupancy_map_server',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        occupancy_map_server_node
    ])
