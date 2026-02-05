from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """
    Launch OctoMap interface demo node
    """
    
    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    octomap_topic = LaunchConfiguration('octomap_topic')
    min_unknown_neighbors = LaunchConfiguration('min_unknown_neighbors')
    n_clusters = LaunchConfiguration('n_clusters')

    octomap_demo = Node(
        package='husky_xarm6_mcr_nbv_planner',
        executable='octomap_interface_demo',
        name='octomap_interface_demo',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'octomap_topic': octomap_topic,
            'visualization_topic': 'nbv_markers',
            'map_frame': 'map',
            'min_unknown_neighbors': min_unknown_neighbors,
            'n_clusters': n_clusters,
            'min_unknown_neighbors': 1,
            'n_clusters': -1,  # Auto-select
            'update_rate_hz': 2.0,
            'visualization_rate_hz': 2.0,
        }],
    )

    return [octomap_demo]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'octomap_topic',
            default_value='/octomap_binary',
            description='Topic to subscribe for octomap messages'
        ),
        DeclareLaunchArgument(
            'min_unknown_neighbors',
            default_value='1',
            description='Minimum number of unknown neighbors for frontier detection'
        ),
        DeclareLaunchArgument(
            'n_clusters',
            default_value='0',
            description='Number of clusters (0 = auto-select based on frontier count)'
        ),
        OpaqueFunction(function=launch_setup),
    ])
