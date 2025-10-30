"""
Launch file for NBV planning system.

Brings up:
- Volumetric mapping node
- NBV planner node
- Visualization tools
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Package path
    pkg_share = FindPackageShare('husky_xarm6_mcr_planning')
    
    # Config file
    config_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'nbv_planner.yaml'
    ])
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Volumetric map node
    volumetric_map_node = Node(
        package='husky_xarm6_mcr_planning',
        executable='volumetric_map_node',
        name='volumetric_map',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # NBV planner node
    nbv_planner_node = Node(
        package='husky_xarm6_mcr_planning',
        executable='nbv_planner_node',
        name='nbv_planner',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # RViz for visualization (optional)
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', PathJoinSubstitution([pkg_share, 'config', 'nbv.rviz'])],
    #     parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    # )
    
    return LaunchDescription([
        use_sim_time_arg,
        volumetric_map_node,
        nbv_planner_node,
        # rviz_node,
    ])
