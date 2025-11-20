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
    use_gazebo = LaunchConfiguration('use_gazebo')
    platform_prefix = LaunchConfiguration('platform_prefix')
    manipulator_prefix = LaunchConfiguration('manipulator_prefix')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_ip = LaunchConfiguration('robot_ip')
    report_type = LaunchConfiguration('report_type')

    description_pkg = get_package_share_directory('husky_xarm6_mcr_description')
    xacro_file = Path(description_pkg) / 'urdf' / 'husky_xarm6_mcr.urdf.xacro'
    control_pkg = get_package_share_directory('husky_xarm6_mcr_control')
    controllers_yaml = Path(control_pkg) / 'config' / 'controllers.yaml'

    # gazebo_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    #     ),
    #     # run (-r), verbose, load empty.sdf
    #     launch_arguments={'gz_args': '-r empty.sdf'}.items()
    # )

    # Process the xacro file
    robot_description = get_xacro_content(
        context,
        xacro_file=xacro_file,
        use_gazebo=use_gazebo,
        manipulator_prefix=manipulator_prefix,
        platform_prefix=platform_prefix,
        robot_ip=robot_ip,
        report_type=report_type
    )

    # Publish the robot state to tf (use simulated time)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}, 
            {'robot_description': robot_description}
        ]
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    platform_velocity_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'platform_velocity_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', str(controllers_yaml)
            ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    xarm6_traj_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'xarm6_traj_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', str(controllers_yaml)
            ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    jsb_then_base = RegisterEventHandler(
        OnProcessExit(target_action=joint_state_broadcaster, on_exit=[platform_velocity_controller, xarm6_traj_controller])
    )

    if use_gazebo.perform(context).lower() == 'true':
        # Use ros_gz_sim entity creation for Ignition Fortress
        spawn_entity = Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-topic', 'robot_description',
                '-name', 'husky_xarm6_mcr',
                '-z', '0.3', # slight z offset to avoid collision with ground plane
                # Initial joint positions to avoid collision
                '-j', 'xarm6_joint1', '0.0',
                '-j', 'xarm6_joint2', '-0.5',
                '-j', 'xarm6_joint3', '0.5',
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        )

        # Control bridge - handles robot control topics only
        # Clock is now handled by the gazebo.launch.py file
        # Example: /cmd_vel@geometry_msgs/msg/Twist means the bridge will expose a ROS topic /cmd_vel with type geometry_msgs/msg/Twist.
        control_bridge = Node(
            name='control_bridge',
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/platform_velocity_controller/cmd_vel_unstamped@geometry_msgs/msg/Twist[gz.msgs.Twist',
                # '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
            ],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        )

        spawn_then_jsb = RegisterEventHandler(
            OnProcessExit(target_action=spawn_entity, on_exit=[joint_state_broadcaster])
        )

        return [
            robot_state_publisher,
            spawn_entity,
            spawn_then_jsb,
            jsb_then_base,
            control_bridge,
        ]
    
    else:
        # Real hardware - need to explicitly start controller_manager
        controller_manager_node = Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_description},
                str(controllers_yaml),
                {'use_sim_time': use_sim_time}
            ],
            output='screen',
        )
        
        # For real hardware, start controller_manager first, then controllers
        cm_then_jsb = RegisterEventHandler(
            OnProcessExit(target_action=controller_manager_node, on_exit=[joint_state_broadcaster])
        )
        
        return [
            robot_state_publisher,
            controller_manager_node,
            cm_then_jsb,
            jsb_then_base,
        ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('manipulator_prefix', default_value='xarm6_', description='Prefix for manipulator joint names'),
        DeclareLaunchArgument('platform_prefix', default_value='a200_', description='Prefix for platform joint names or frames'),
        DeclareLaunchArgument('use_gazebo', default_value='false', description='Enable simulation mode'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulated time if true'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.205', description='IP address of the xArm6 robot'),
        DeclareLaunchArgument('report_type', default_value='normal', description='Report type for xArm (normal, rich, dev)'),
        OpaqueFunction(function=launch_setup)
    ])