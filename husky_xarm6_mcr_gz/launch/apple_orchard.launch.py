import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg_orchard = get_package_share_directory('husky_xarm6_mcr_gz')  # or 'orchard_worlds'
    worlds_dir = os.path.join(pkg_orchard, 'worlds')
    models_dir = os.path.join(pkg_orchard, 'models')  # if you ship models/ too

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

    return LaunchDescription([
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
                'gz_args': ['-r ', PathJoinSubstitution([pkg_orchard, 'worlds', 'apple_orchard.sdf'])]
            }.items()
        ),
    ])
