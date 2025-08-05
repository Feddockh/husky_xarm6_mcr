# bringup_server.launch.py
#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription

def launch_robot(context, *args, **kwargs):
    fake = LaunchConfiguration('fake').perform(context)
    prefix = LaunchConfiguration('prefix').perform(context)
    hw_ns = LaunchConfiguration('hw_ns').perform(context)

    if fake == "true":
        pkg = 'xarm_moveit_config'
        file = '_robot_moveit_fake.launch.py'
        args = {
            'prefix': prefix,
            'dof': '6',
            'robot_type': 'xarm',
            'hw_ns': hw_ns,
            'no_gui_ctrl': 'true'
        }
    else:
        pkg = 'xarm_moveit_config'
        file = '_robot_moveit_realmove.launch.py'
        args = {
            'robot_ip': LaunchConfiguration('robot_ip'),
            'prefix': prefix,
            'dof': '6',
            'robot_type': 'xarm',
            'hw_ns': hw_ns,
            'no_gui_ctrl': 'true'
        }

    return [
      IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
          PathJoinSubstitution([ FindPackageShare(pkg), 'launch', file ])
        ),
        launch_arguments=args.items()
      )
    ]

def generate_launch_description():
    return LaunchDescription([
      DeclareLaunchArgument('fake',    default_value='true'),
      DeclareLaunchArgument('robot_ip',default_value='192.168.1.205'),
      DeclareLaunchArgument('hw_ns',   default_value='xarm'),
      DeclareLaunchArgument('prefix',  default_value=''),
      OpaqueFunction(function=launch_robot)
    ])
