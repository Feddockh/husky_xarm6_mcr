"""
Launch file for MoveIt-integrated occupancy map server
This publishes octomap_msgs/Octomap to /octomap_binary for MoveIt planning scene
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('husky_xarm6_mcr_occupancy_map'),
            'config',
            'occupancy_map_params.yaml'
        ]),
        description='Path to occupancy map configuration file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Occupancy map parameters
    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.05',
        description='Octomap resolution in meters'
    )

    max_range_arg = DeclareLaunchArgument(
        'max_range',
        default_value='5.0',
        description='Maximum sensor range in meters'
    )

    min_range_arg = DeclareLaunchArgument(
        'min_range',
        default_value='0.3',
        description='Minimum sensor range in meters'
    )

    prob_hit_arg = DeclareLaunchArgument(
        'prob_hit',
        default_value='0.7',
        description='Probability of occupancy when sensor detects obstacle'
    )

    prob_miss_arg = DeclareLaunchArgument(
        'prob_miss',
        default_value='0.4',
        description='Probability of occupancy when ray passes through voxel'
    )

    clamp_min_arg = DeclareLaunchArgument(
        'clamp_min',
        default_value='0.12',
        description='Minimum clamping threshold for log-odds'
    )

    clamp_max_arg = DeclareLaunchArgument(
        'clamp_max',
        default_value='0.97',
        description='Maximum clamping threshold for log-odds'
    )

    occupancy_threshold_arg = DeclareLaunchArgument(
        'occupancy_threshold',
        default_value='0.5',
        description='Threshold for classifying voxels as occupied'
    )

    map_frame_arg = DeclareLaunchArgument(
        'map_frame',
        default_value='map',
        description='Fixed reference frame for the occupancy map'
    )

    filter_ground_plane_arg = DeclareLaunchArgument(
        'filter_ground_plane',
        default_value='true',
        description='Enable ground plane filtering'
    )

    ground_distance_threshold_arg = DeclareLaunchArgument(
        'ground_distance_threshold',
        default_value='0.04',
        description='Distance threshold for ground plane filtering in meters'
    )

    pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='/firefly_left/points2',
        description='Point cloud input topic'
    )

    octomap_topic_arg = DeclareLaunchArgument(
        'octomap_topic',
        default_value='/octomap_binary',
        description='Topic to publish octomap for MoveIt planning scene'
    )

    publish_free_voxels_arg = DeclareLaunchArgument(
        'publish_free_voxels',
        default_value='false',
        description='Publish free space voxels in visualization'
    )

    visualization_rate_arg = DeclareLaunchArgument(
        'visualization_rate',
        default_value='2.0',
        description='Rate for publishing visualization markers in Hz'
    )

    octomap_publish_rate_arg = DeclareLaunchArgument(
        'octomap_publish_rate',
        default_value='1.0',
        description='Rate for publishing octomap messages in Hz'
    )

    # MoveIt octomap server node
    moveit_octomap_server_node = Node(
        package='husky_xarm6_mcr_occupancy_map',
        executable='moveit_octomap_server',
        name='moveit_octomap_server',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'resolution': LaunchConfiguration('resolution'),
                'max_range': LaunchConfiguration('max_range'),
                'min_range': LaunchConfiguration('min_range'),
                'prob_hit': LaunchConfiguration('prob_hit'),
                'prob_miss': LaunchConfiguration('prob_miss'),
                'clamp_min': LaunchConfiguration('clamp_min'),
                'clamp_max': LaunchConfiguration('clamp_max'),
                'occupancy_threshold': LaunchConfiguration('occupancy_threshold'),
                'map_frame': LaunchConfiguration('map_frame'),
                'filter_ground_plane': LaunchConfiguration('filter_ground_plane'),
                'ground_distance_threshold': LaunchConfiguration('ground_distance_threshold'),
                'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
                'octomap_topic': LaunchConfiguration('octomap_topic'),
                'publish_free_voxels': LaunchConfiguration('publish_free_voxels'),
                'visualization_rate': LaunchConfiguration('visualization_rate'),
                'octomap_publish_rate': LaunchConfiguration('octomap_publish_rate'),
            }
        ],
        remappings=[
            # Add any topic remappings here if needed
        ]
    )

    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        resolution_arg,
        max_range_arg,
        min_range_arg,
        prob_hit_arg,
        prob_miss_arg,
        clamp_min_arg,
        clamp_max_arg,
        occupancy_threshold_arg,
        map_frame_arg,
        filter_ground_plane_arg,
        ground_distance_threshold_arg,
        pointcloud_topic_arg,
        octomap_topic_arg,
        publish_free_voxels_arg,
        visualization_rate_arg,
        octomap_publish_rate_arg,
        moveit_octomap_server_node
    ])
