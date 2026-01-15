from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
    RegisterEventHandler,
    Shutdown,
)
from launch.actions import RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    """
    Main bringup orchestration:
    1. Start Gazebo environment (apple orchard or empty world) if use_gazebo=true
    2. Spawn the robot and start ros2_control
    3. Bring up MoveIt move_group
    4. Optionally launch RViz
    
    Logic:
    - If use_gazebo=true, then use_fake_hardware is forced to false
    - If use_gazebo=false, then use_fake_hardware can be true or false
    """
    
    # Get launch configurations
    use_gazebo = LaunchConfiguration('use_gazebo')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    use_rviz = LaunchConfiguration('use_rviz')
    manipulator_prefix = LaunchConfiguration('manipulator_prefix')
    manipulator_ns = LaunchConfiguration('manipulator_ns')
    platform_prefix = LaunchConfiguration('platform_prefix')
    platform_ns = LaunchConfiguration('platform_ns')
    robot_ip = LaunchConfiguration('robot_ip')
    report_type = LaunchConfiguration('report_type')
    
    # Get package directories
    bringup_pkg = get_package_share_directory('husky_xarm6_mcr_bringup')
    gz_pkg = get_package_share_directory('husky_xarm6_mcr_gz')
    control_pkg = get_package_share_directory('husky_xarm6_mcr_control')
    moveit_pkg = get_package_share_directory('husky_xarm6_mcr_moveit_config')
    
    launch_actions = []
    
    # Validate logic: if use_gazebo=true, force use_fake_hardware=false
    use_gazebo_val = use_gazebo.perform(context).lower() == 'true'
    use_fake_hardware_val = use_fake_hardware.perform(context).lower() == 'true'
    
    if use_gazebo_val and use_fake_hardware_val:
        print("[WARNING] use_gazebo=true and use_fake_hardware=true is invalid. Forcing use_fake_hardware=false.")
        use_fake_hardware_val = False
    
    # Convert back to string for passing to launch arguments
    use_fake_hardware_str = 'true' if use_fake_hardware_val else 'false'
    
    # Note: map_frame_publisher is now conditionally launched based on use_localization
    # (see below after MoveIt launch)
    
    if use_gazebo_val:
        # Launch Gazebo with the selected world using the general gazebo.launch.py
        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gz_pkg, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': world,
            }.items()
        )
        launch_actions.append(gazebo_launch)
        
        # Launch ros2_control after a delay to ensure Gazebo is ready
        # This prevents the gz_ros2_control plugin from auto-spawning before we're ready
        control_launch = TimerAction(
            period=3.0,  # 3 second delay
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(control_pkg, 'launch', 'ros2_control.launch.py')
                    ),
                    launch_arguments={
                        'use_gazebo': use_gazebo,
                        'use_fake_hardware': use_fake_hardware_str,
                        'use_sim_time': use_sim_time,
                        'manipulator_prefix': manipulator_prefix,
                        'manipulator_ns': manipulator_ns,
                        'platform_prefix': platform_prefix,
                        'platform_ns': platform_ns,
                        'robot_ip': robot_ip,
                        'report_type': report_type,
                    }.items()
                )
            ]
        )
        launch_actions.append(control_launch)

    # No Gazebo - launch ros2_control immediately without delay
    else:
        control_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(control_pkg, 'launch', 'ros2_control.launch.py')
            ),
            launch_arguments={
                'use_gazebo': use_gazebo,
                'use_fake_hardware': use_fake_hardware_str,
                'use_sim_time': use_sim_time,
                'manipulator_prefix': manipulator_prefix,
                'manipulator_ns': manipulator_ns,
                'platform_prefix': platform_prefix,
                'platform_ns': platform_ns,
                'robot_ip': robot_ip,
                'report_type': report_type,
            }.items()
        )
        launch_actions.append(control_launch)

    # Simple odometry publisher could be added here
    
    # Launch MoveIt move_group
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_pkg, 'launch', 'move_group.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_gazebo': use_gazebo,
            'manipulator_prefix': manipulator_prefix,
            'platform_prefix': platform_prefix,
        }.items()
    )
    launch_actions.append(moveit_launch)

    # Republish odom to map if localization
    # Only bringup the map if the gazebo is used because odom is only being grabbed from the gazebo simulation
    if use_gazebo_val or use_fake_hardware_val:
        map_frame_publisher = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen'
        )
        launch_actions.append(map_frame_publisher)
    
    # Optionally launch RViz
    if use_rviz.perform(context).lower() == 'true':
        rviz_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(moveit_pkg, 'launch', 'rviz.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'manipulator_prefix': manipulator_prefix,
                'platform_prefix': platform_prefix,
            }.items()
        )
        launch_actions.append(rviz_launch)

    # Bringup the stereo camera
    # Set up the stereo camera bridges
    # Image bridge for camera images only
    # image_bridge = Node(
    #     name='image_bridge',
    #     package='ros_gz_image',
    #     executable='image_bridge',
    #     arguments=[
    #         '/firefly_left/image_raw',
    #         '/firefly_right/image_raw',
    #     ],
    #     output='screen'
    # )
    # launch_actions.append(image_bridge)
    
    # # Generic bridge for camera_info
    # camera_info_bridge = Node(
    #     name='camera_info_bridge',
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     arguments=[
    #         '/firefly_left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
    #         '/firefly_right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
    #     ],
    #     output='screen'
    # )
    # launch_actions.append(camera_info_bridge)
    
    return launch_actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gazebo',
            default_value='false',
            description='Whether to use Gazebo simulation'
        ),
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='Use fake hardware interface if true'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='empty',
            description='World to load: "apple_orchard", "empty", or path to .sdf file'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to launch RViz'
        ),
        DeclareLaunchArgument(
            'manipulator_prefix',
            default_value='xarm6_',
            description='Prefix for manipulator joint names'
        ),
        DeclareLaunchArgument(
            'manipulator_ns',
            default_value='xarm',
            description='Namespace for manipulator'
        ),
        DeclareLaunchArgument(
            'platform_prefix',
            default_value='a200_',
            description='Prefix for platform joint names or frames'
        ),
        DeclareLaunchArgument(
            'platform_ns',
            default_value='husky',
            description='Namespace for platform'
        ),
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.1.205',
            description='IP address of the xArm6 robot'
        ),
        DeclareLaunchArgument(
            'report_type',
            default_value='normal',
            description='Report type for xArm (normal, rich, dev)'
        ),
        OpaqueFunction(function=launch_setup)
    ])
