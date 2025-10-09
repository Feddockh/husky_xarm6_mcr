from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    """
    Main bringup orchestration:
    1. Start Gazebo environment (apple orchard or empty world)
    2. Spawn the robot and start ros2_control
    3. Bring up MoveIt move_group
    4. Optionally launch RViz
    """
    
    # Get launch configurations
    world = LaunchConfiguration('world')
    use_rviz = LaunchConfiguration('use_rviz')
    manipulator_prefix = LaunchConfiguration('manipulator_prefix')
    platform_prefix = LaunchConfiguration('platform_prefix')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Get package directories
    gz_pkg = get_package_share_directory('husky_xarm6_mcr_gz')
    control_pkg = get_package_share_directory('husky_xarm6_mcr_control')
    moveit_pkg = get_package_share_directory('husky_xarm6_mcr_moveit_config')
    
    # Determine which world to load
    world_value = world.perform(context)
    
    launch_actions = []
    
    # 1. Launch Gazebo with the selected world
    if world_value == 'apple_orchard':
        # Use the custom apple orchard launch
        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gz_pkg, 'launch', 'apple_orchard.launch.py')
            )
        )
        launch_actions.append(gazebo_launch)
    else:
        # For empty world or custom world, use direct ign gazebo command
        from launch.actions import ExecuteProcess
        
        if world_value == 'empty':
            world_file = 'empty.sdf'
        else:
            # Assume it's a path to a world file
            world_file = world_value
        
        gazebo_launch = ExecuteProcess(
            cmd=['ign', 'gazebo', world_file, '-v', '4'],
            output='screen'
        )
        launch_actions.append(gazebo_launch)
    
    # 2. Launch ros2_control after a delay to ensure Gazebo is ready
    # This prevents the gz_ros2_control plugin from auto-spawning before we're ready
    control_launch = TimerAction(
        period=3.0,  # 3 second delay
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(control_pkg, 'launch', 'ros2_control.launch.py')
                ),
                launch_arguments={
                    'manipulator_prefix': manipulator_prefix,
                    'platform_prefix': platform_prefix,
                    'sim': 'true',
                }.items()
            )
        ]
    )
    launch_actions.append(control_launch)
    
    # 3. Launch MoveIt move_group
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_pkg, 'launch', 'move_group.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'manipulator_prefix': manipulator_prefix,
            'platform_prefix': platform_prefix,
        }.items()
    )
    launch_actions.append(moveit_launch)
    
    # 4. Optionally launch RViz
    use_rviz_value = use_rviz.perform(context)
    if use_rviz_value.lower() == 'true':
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
    
    return launch_actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='apple_orchard',
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
            'platform_prefix',
            default_value='a200_',
            description='Prefix for platform joint names or frames'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        OpaqueFunction(function=launch_setup)
    ])
