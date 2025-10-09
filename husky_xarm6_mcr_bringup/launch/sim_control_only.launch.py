from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Minimal bringup for simulation without MoveIt.
    Useful for testing controllers or platform navigation only.
    
    This launches:
    1. Gazebo with selected world
    2. Robot with ros2_control
    """
    
    # Get package directories
    gz_pkg = get_package_share_directory('husky_xarm6_mcr_gz')
    control_pkg = get_package_share_directory('husky_xarm6_mcr_control')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='apple_orchard',
            description='World to load: "apple_orchard" or "empty"'
        ),
        DeclareLaunchArgument(
            'manipulator_prefix',
            default_value='xarm6_',
            description='Prefix for manipulator joint names'
        ),
        DeclareLaunchArgument(
            'platform_prefix',
            default_value='a200_',
            description='Prefix for platform joint names or frames'
        ),
        
        # Launch Gazebo with apple orchard
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gz_pkg, 'launch', 'apple_orchard.launch.py')
            )
        ),
        
        # Launch ros2_control (spawns robot and controllers)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(control_pkg, 'launch', 'ros2_control.launch.py')
            ),
            launch_arguments={
                'manipulator_prefix': LaunchConfiguration('manipulator_prefix'),
                'platform_prefix': LaunchConfiguration('platform_prefix'),
                'sim': 'true',
            }.items()
        ),
    ])
