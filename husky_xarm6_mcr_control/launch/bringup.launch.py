from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    this_dir = get_package_share_directory('husky_xarm6_mcr_control')
    control_launch = os.path.join(this_dir, 'launch', 'ros2_control.launch.py')

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(control_launch),
            launch_arguments={
                'manipulator_prefix': LaunchConfiguration('manipulator_prefix'),
                'platform_prefix': LaunchConfiguration('platform_prefix'),
                'sim': LaunchConfiguration('sim')
            }.items()
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('manipulator_prefix', default_value='xarm6_', description='Prefix forwarded to manipulator launches'),
        DeclareLaunchArgument('platform_prefix', default_value='a200_', description='Prefix forwarded to platform launches'),
        DeclareLaunchArgument('sim', default_value='true', description='Enable simulation mode'),
        OpaqueFunction(function=launch_setup)
    ])
