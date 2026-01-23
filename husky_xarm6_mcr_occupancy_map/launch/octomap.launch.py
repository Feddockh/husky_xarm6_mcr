"""
Launch file for occupancy map server
Optionally publishes to MoveIt planning scene if use_moveit is enabled
Allows switching between PointCloudUpdater and DepthImageUpdater
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # --- Core args ---
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',  # real-world default
        description='Use simulation time'
    )

    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.02',
        description='Octomap resolution in meters'
    )

    max_range_arg = DeclareLaunchArgument(
        'max_range',
        default_value='3.0',
        description='Maximum sensor range in meters'
    )

    use_moveit_arg = DeclareLaunchArgument(
        'use_moveit',
        default_value='false',
        description='Enable MoveIt planning scene integration'
    )

    use_bbox_arg = DeclareLaunchArgument(
        'use_bbox',
        default_value='true',
        description='Enable bounding box for octomap updates'
    )

    map_frame_arg = DeclareLaunchArgument(
        'map_frame',
        default_value='map',
        description='Frame for the occupancy map'
    )

    # --- Updater selection ---
    updater_type_arg = DeclareLaunchArgument(
        'updater_type',
        default_value='pointcloud',
        description="Updater type: 'pointcloud' or 'depth_image'"
    )

    # --- PointCloud updater topic ---
    pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='/firefly_left/points2',
        description='PointCloud2 topic for PointCloudUpdater'
    )

    # --- Depth image updater topics ---
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/firefly_left/depth',
        description='Depth image topic (32FC1 meters or 16UC1) for DepthImageUpdater'
    )

    depth_info_topic_arg = DeclareLaunchArgument(
        'depth_info_topic',
        default_value='/firefly_left/camera_info_rect_scaled',
        description='CameraInfo topic matching the depth image'
    )

    # --- Depth updater behavior knobs ---
    depth_stride_arg = DeclareLaunchArgument(
        'depth_stride',
        default_value='2',
        description='Pixel stride for DepthImageUpdater (2/4/8 to reduce CPU)'
    )

    clear_no_return_arg = DeclareLaunchArgument(
        'clear_no_return',
        default_value='true',
        description='If true: pixels with invalid depth clear free space to max_range'
    )

    depth_scale_arg = DeclareLaunchArgument(
        'depth_scale',
        default_value='0.001',
        description='Meters per unit for 16UC1 depth (mm->m = 0.001). Ignored for 32FC1.'
    )

    # Octomap server node with inline parameters
    octomap_server_node = Node(
        package='husky_xarm6_mcr_occupancy_map',
        executable='octomap_server',
        name='octomap_server',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),

            # Octomap parameters
            'resolution': LaunchConfiguration('resolution'),
            'map_frame': LaunchConfiguration('map_frame'),

            # Sensor model
            'max_range': LaunchConfiguration('max_range'),
            'min_range': 0.01,
            'prob_hit': 0.7,
            'prob_miss': 0.4,
            'clamp_min': 0.12,
            'clamp_max': 0.97,
            'occupancy_threshold': 0.5,

            # Ground filtering
            'filter_ground_plane': True,
            'ground_distance_threshold': 0.04,

            # Bounding box (limits octomap updates to this region)
            'use_bounding_box': LaunchConfiguration('use_bbox'),
            'bbx_min_x': -1.0,
            'bbx_min_y': -2.0,
            'bbx_min_z': 0.0,
            'bbx_max_x': 1.0,
            'bbx_max_y': -0.5,
            'bbx_max_z': 2.0,

            # --- Updater selection + topics ---
            'updater_type': LaunchConfiguration('updater_type'),

            # PointCloud updater
            'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),

            # Depth image updater
            'depth_topic': LaunchConfiguration('depth_topic'),
            'depth_info_topic': LaunchConfiguration('depth_info_topic'),

            # Depth updater tuning (names match the DepthImageUpdater params I suggested)
            'depth_updater.stride': LaunchConfiguration('depth_stride'),
            'depth_updater.clear_no_return': LaunchConfiguration('clear_no_return'),
            'depth_updater.depth_scale': LaunchConfiguration('depth_scale'),

            # MoveIt + publishing
            'use_moveit': LaunchConfiguration('use_moveit'),
            'planning_scene_world_topic': '/planning_scene_world',

            'publish_free_voxels': True,
            'enable_visualization': True,
            'visualization_topic': 'occupancy_map_markers',
            'visualization_rate': 1.0,
            'octomap_publish_rate': 1.0,
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        resolution_arg,
        max_range_arg,
        use_moveit_arg,
        use_bbox_arg,
        map_frame_arg,

        updater_type_arg,
        pointcloud_topic_arg,
        depth_topic_arg,
        depth_info_topic_arg,
        depth_stride_arg,
        clear_no_return_arg,
        depth_scale_arg,

        octomap_server_node
    ])
