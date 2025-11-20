from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch_param_builder import load_xacro
import os
from pathlib import Path
from launch.actions import OpaqueFunction

# From xarm packages
def get_xacro_content(context, xacro_file, **kwargs):
    xacro_file = Path(xacro_file.perform(context)) if isinstance(xacro_file, LaunchConfiguration) else Path(xacro_file) if isinstance(xacro_file, str) else xacro_file
    
    def get_param_str(param):
        val = param if isinstance(param, str) else 'false' if param == False else 'true' if param == True else (param.perform(context) if context is not None else param) if isinstance(param, LaunchConfiguration) else str(param)
        return val if not val else val[1:-1] if isinstance(val, str) and val[0] in ['"', '\''] and val[-1] in ['"', '\''] else val

    mappings = {}
    for k, v in kwargs.items():
        mappings[k] = get_param_str(v)
    return load_xacro(xacro_file, mappings=mappings)

def launch_setup(context, *args, **kwargs):
    use_gazebo = LaunchConfiguration('use_gazebo', default='false')
    manipulator_prefix = LaunchConfiguration('manipulator_prefix', default='xarm6_')
    platform_prefix = LaunchConfiguration('platform_prefix', default='a200_')

    description_pkg = get_package_share_directory('husky_xarm6_mcr_description')
    urdf_file = Path(description_pkg) / 'urdf' / 'husky_xarm6_mcr.urdf.xacro'

    robot_description = get_xacro_content(
        context,
        xacro_file=urdf_file,
        use_gazebo=use_gazebo,
        manipulator_prefix=manipulator_prefix,
        platform_prefix=platform_prefix
    )
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(description_pkg, 'rviz', 'view.rviz')],
        output='screen'
    )

    return [
        robot_state_publisher_node, 
        joint_state_publisher_node, 
        rviz_node
    ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])