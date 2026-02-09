from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    """
    Launch OctoMap interface demo node
    """
    
    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    octomap_topic = LaunchConfiguration('octomap_topic')
    min_unknown_neighbors = LaunchConfiguration('min_unknown_neighbors')
    n_clusters = LaunchConfiguration('n_clusters')
    gt_points_file = LaunchConfiguration('gt_points_file')
    enable_evaluation = LaunchConfiguration('enable_evaluation')
    eval_threshold_radius = LaunchConfiguration('eval_threshold_radius')

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
            'use_bbox': True,
            'update_rate_hz': 1.0,
            'gt_points_file': gt_points_file,
            'enable_evaluation': enable_evaluation,
            'eval_threshold_radius': eval_threshold_radius,
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
        DeclareLaunchArgument(
            'gt_points_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('husky_xarm6_mcr_nbv_planner'),
                'metrics', 'gt_points', 'sim_aruco_gt_points.yaml'
            ]),
            description='Path to the ground truth points YAML file for semantic evaluation'
        ),
        DeclareLaunchArgument(
            'enable_evaluation',
            default_value='true',
            description='Enable semantic octomap evaluation against ground truth'
        ),
        DeclareLaunchArgument(
            'eval_threshold_radius',
            default_value='0.1',
            description='Threshold radius (meters) for matching clusters to ground truth points'
        ),
        OpaqueFunction(function=launch_setup),
    ])
