import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    """
    Launch Gazebo with the specified world.
    Sets up resource paths for custom models and worlds.
    """
    world = LaunchConfiguration('world')
    world_value = world.perform(context)
    
    pkg_gz = get_package_share_directory('husky_xarm6_mcr_gz')
    worlds_dir = os.path.join(pkg_gz, 'worlds')
    models_dir = os.path.join(pkg_gz, 'models')

    # All currently-sourced ROS packages' share dirs
    ament_prefix = os.environ.get('AMENT_PREFIX_PATH', '')
    packages_paths = [os.path.join(p, 'share') for p in ament_prefix.split(':') if p]

    # Existing resource paths (if any)
    ign_existing = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    gz_existing  = os.environ.get('GZ_SIM_RESOURCE_PATH', '')

    # Compose new paths: put our package first, then all ROS shares, then previous value
    ign_path = ':'.join(
        [worlds_dir, models_dir] +
        packages_paths +
        ([ign_existing] if ign_existing else [])
    )
    gz_path = ':'.join(
        [worlds_dir, models_dir] +
        packages_paths +
        ([gz_existing] if gz_existing else [])
    )
    
    # Determine the world file to load
    if world_value in ['apple_orchard', 'empty']:
        # Use one of our custom worlds
        world_file = PathJoinSubstitution([pkg_gz, 'worlds', f'{world_value}.sdf'])
    elif os.path.isabs(world_value):
        # Absolute path provided
        world_file = world_value
    else:
        # Assume it's a relative path from our worlds directory
        world_file = PathJoinSubstitution([pkg_gz, 'worlds', world_value])

    return [
        # Set for Ignition Fortress
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=ign_path
        ),
        # Set for newer Gazebo Sim (gz sim)
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=gz_path
        ),

        # Launch Gazebo with ros_gz_sim (properly sets up plugin paths)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': ['-r ', world_file]
            }.items()
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='apple_orchard',
            description='World to load: "empty", "apple_orchard", custom world name, or absolute path to .sdf file'
        ),
        OpaqueFunction(function=launch_setup)
    ])
