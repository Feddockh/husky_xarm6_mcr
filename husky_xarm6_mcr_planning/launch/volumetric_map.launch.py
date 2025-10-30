from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

def generate_launch_description():
    # --- Launch arguments ---
    pointcloud_topic = LaunchConfiguration('pointcloud_topic')
    map_frame        = LaunchConfiguration('map_frame')
    resolution       = LaunchConfiguration('resolution')
    map_size_x       = LaunchConfiguration('map_size_x')
    map_size_y       = LaunchConfiguration('map_size_y')
    map_size_z       = LaunchConfiguration('map_size_z')
    origin_x         = LaunchConfiguration('origin_x')
    origin_y         = LaunchConfiguration('origin_y')
    origin_z         = LaunchConfiguration('origin_z')
    max_depth        = LaunchConfiguration('max_depth')
    downsample_voxel = LaunchConfiguration('downsample_voxel')
    rebuild_every_n  = LaunchConfiguration('rebuild_every_n')
    publish_accum    = LaunchConfiguration('publish_accumulated_cloud')

    cam_fx           = LaunchConfiguration('cam_fx')
    cam_fy           = LaunchConfiguration('cam_fy')
    cam_cx           = LaunchConfiguration('cam_cx')
    cam_cy           = LaunchConfiguration('cam_cy')
    cam_width        = LaunchConfiguration('cam_width')
    cam_height       = LaunchConfiguration('cam_height')
    cam_max_range    = LaunchConfiguration('cam_max_range')
    cam_pixel_stride = LaunchConfiguration('cam_pixel_stride')

    use_rviz         = LaunchConfiguration('rviz')
    # Optional: use a packaged RViz config if you add one later
    pkg_share = get_package_share_directory('husky_xarm6_mcr_planning')
    default_rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'view.rviz'])

    declares = [
        DeclareLaunchArgument('pointcloud_topic', default_value='/firefly_left/points2',
                              description='Input PointCloud2 topic'),
        DeclareLaunchArgument('map_frame', default_value='map',
                              description='Fixed frame for the map'),
        DeclareLaunchArgument('resolution', default_value='0.05',
                              description='Voxel size (m)'),
        DeclareLaunchArgument('map_size_x', default_value='10.0'),
        DeclareLaunchArgument('map_size_y', default_value='10.0'),
        DeclareLaunchArgument('map_size_z', default_value='5.0'),
        DeclareLaunchArgument('origin_x', default_value='-5.0'),
        DeclareLaunchArgument('origin_y', default_value='-5.0'),
        DeclareLaunchArgument('origin_z', default_value='0.0'),
        DeclareLaunchArgument('max_depth', default_value='8',
                              description='Open3D octree max depth'),
        DeclareLaunchArgument('downsample_voxel', default_value='0.05',
                              description='Downsample size (m) for incoming clouds (0 to disable)'),
        DeclareLaunchArgument('rebuild_every_n', default_value='5',
                              description='Rebuild octree every N updates'),
        DeclareLaunchArgument('publish_accumulated_cloud', default_value='true',
                              description='Publish accumulated PointCloud2 for debugging'),

        # Camera intrinsics/extrinsics for IG
        DeclareLaunchArgument('cam_fx', default_value='500.0'),
        DeclareLaunchArgument('cam_fy', default_value='500.0'),
        DeclareLaunchArgument('cam_cx', default_value='320.0'),
        DeclareLaunchArgument('cam_cy', default_value='240.0'),
        DeclareLaunchArgument('cam_width', default_value='640'),
        DeclareLaunchArgument('cam_height', default_value='480'),
        DeclareLaunchArgument('cam_max_range', default_value='3.0'),
        DeclareLaunchArgument('cam_pixel_stride', default_value='16'),

        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz2 with a basic config'),
    ]

    # --- Volumetric map node ---
    volumetric_map_node = Node(
        package='husky_xarm6_mcr_planning',
        executable='volumetric_map_node',  # If no console_script, use something like: executable='python3', arguments=['-m','husky_xarm6_mcr_planning.perception.volumetric_map']
        name='volumetric_map_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,  # Use Gazebo simulation time
            'pointcloud_topic': pointcloud_topic,
            'map_frame': map_frame,
            'resolution': resolution,
            'map_size_x': map_size_x,
            'map_size_y': map_size_y,
            'map_size_z': map_size_z,
            'origin_x': origin_x,
            'origin_y': origin_y,
            'origin_z': origin_z,
            'max_depth': max_depth,
            'downsample_voxel': downsample_voxel,
            'rebuild_octree_every_n': rebuild_every_n,
            'publish_accumulated_cloud': publish_accum,

            'camera.fx': cam_fx,
            'camera.fy': cam_fy,
            'camera.cx': cam_cx,
            'camera.cy': cam_cy,
            'camera.width': cam_width,
            'camera.height': cam_height,
            'camera.max_range': cam_max_range,
            'camera.pixel_stride': cam_pixel_stride,
        }]
    )

    # --- RViz2 (optional) ---
    # Minimal config: shows Marker and PointCloud2 under fixed frame
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription(declares + [volumetric_map_node, rviz2])
