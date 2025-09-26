from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch_param_builder import load_xacro
import os
from pathlib import Path
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import SetEnvironmentVariable


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
    sim = LaunchConfiguration('sim')
    platform_prefix = LaunchConfiguration('platform_prefix')
    manipulator_prefix = LaunchConfiguration('manipulator_prefix')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    description_pkg = get_package_share_directory('husky_xarm6_mcr_description')
    xacro_file = Path(description_pkg) / 'urdf' / 'husky_xarm6_mcr.urdf.xacro'
    control_pkg = get_package_share_directory('husky_xarm6_mcr_control')
    controllers_yaml = Path(control_pkg) / 'config' / 'controllers.yaml'

    # Start the Ignition/Gazebo (Fortress) server via ros_ign_gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        # run (-r), verbose, load empty.sdf
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # Process the xacro file
    robot_description = get_xacro_content(
        context,
        xacro_file=xacro_file,
        sim=sim,
        manipulator_prefix=manipulator_prefix,
        platform_prefix=platform_prefix
    )

    # Publish the robot state to tf (use simulated time)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}, {'robot_description': robot_description}]
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    platform_velocity_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'platform_velocity_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', str(controllers_yaml)
            ],
    )

    xarm6_traj_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'xarm6_traj_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', str(controllers_yaml)
            ],
    )

    # Use ros_gz_sim entity creation for Ignition Fortress
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'husky_xarm6_mcr',
            '-z', '0.3' # slight z offset to avoid collision with ground plane
        ],
        parameters=[{'use_sim_time': True}],
    )

    # The [ means GZ → ROS (subscribe to GZ /clock, publish to ROS /clock). Avoid ROS→GZ /clock to prevent loops.
    # Example: /cmd_vel@geometry_msgs/msg/Twist means the bridge will expose a ROS topic /cmd_vel with type geometry_msgs/msg/Twist.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/platform_velocity_controller/cmd_vel_unstamped@geometry_msgs/msg/Twist[gz.msgs.Twist'
        ],
        output='screen'
    )

    spawn_then_jsb = RegisterEventHandler(
        OnProcessExit(target_action=spawn_entity, on_exit=[joint_state_broadcaster])
    )
    jsb_then_base = RegisterEventHandler(
        OnProcessExit(target_action=joint_state_broadcaster, on_exit=[platform_velocity_controller, xarm6_traj_controller])
    )


    # arm_controller_spawner = RegisterEventHandler(
    #     OnProcessExit(
    #         target_action=joint_state_broadcaster,
    #         on_exit=[arm_controller]
    #     )
    # )

    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', os.path.join(description_pkg, 'rviz', 'view.rviz')],
    #     output='screen'
    # )

    return [
        gazebo_launch,
        robot_state_publisher,
        spawn_entity,
        spawn_then_jsb,
        jsb_then_base,
        bridge,
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('manipulator_prefix', default_value='xarm6_', description='Prefix for manipulator joint names'),
        DeclareLaunchArgument('platform_prefix', default_value='a200_', description='Prefix for platform joint names or frames'),
        DeclareLaunchArgument('sim', default_value='true', description='Enable simulation mode'),
        OpaqueFunction(function=launch_setup)
    ])