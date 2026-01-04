"""
Launch file for MoveIt-integrated occupancy map server
This publishes octomap_msgs/Octomap to /octomap_binary for MoveIt planning scene
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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

    # MoveIt octomap server node
    moveit_octomap_server_node = Node(
        package='husky_xarm6_mcr_occupancy_map',
        executable='moveit_octomap_server',
        name='moveit_octomap_server',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        moveit_octomap_server_node
    ])
