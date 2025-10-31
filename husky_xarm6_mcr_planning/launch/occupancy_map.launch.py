from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

def generate_launch_description():
    """Launch probabilistic occupancy mapping node with RViz."""
    
    # Launch arguments
    pointcloud_topic = LaunchConfiguration('pointcloud_topic')
    map_frame = LaunchConfiguration('map_frame')
    resolution = LaunchConfiguration('resolution')
    max_depth = LaunchConfiguration('max_depth')
    downsample_voxel = LaunchConfiguration('downsample_voxel')
    
    # Occupancy probabilities
    prob_hit = LaunchConfiguration('prob_hit')
    prob_miss = LaunchConfiguration('prob_miss')
    prob_prior = LaunchConfiguration('prob_prior')
    occupied_threshold = LaunchConfiguration('occupied_threshold')
    free_threshold = LaunchConfiguration('free_threshold')
    
    # Visualization
    publish_markers = LaunchConfiguration('publish_markers')
    publish_pc = LaunchConfiguration('publish_point_cloud')
    use_rviz = LaunchConfiguration('rviz')
    
    # Camera params
    cam_fx = LaunchConfiguration('cam_fx')
    cam_fy = LaunchConfiguration('cam_fy')
    cam_cx = LaunchConfiguration('cam_cx')
    cam_cy = LaunchConfiguration('cam_cy')
    cam_width = LaunchConfiguration('cam_width')
    cam_height = LaunchConfiguration('cam_height')
    cam_max_range = LaunchConfiguration('cam_max_range')
    cam_pixel_stride = LaunchConfiguration('cam_pixel_stride')
    
    pkg_share = get_package_share_directory('husky_xarm6_mcr_planning')
    default_rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'view.rviz'])
    
    declares = [
        DeclareLaunchArgument('pointcloud_topic', default_value='/firefly_left/points2',
                              description='Input PointCloud2 topic'),
        DeclareLaunchArgument('map_frame', default_value='map',
                              description='Fixed frame for the map'),
        DeclareLaunchArgument('resolution', default_value='0.1',
                              description='Voxel size (m) at finest resolution'),
        DeclareLaunchArgument('max_depth', default_value='16',
                              description='Maximum octree depth'),
        DeclareLaunchArgument('downsample_voxel', default_value='0.1',
                              description='Downsample size (m) for incoming clouds'),
        
        # Occupancy probabilities (Octomap defaults)
        DeclareLaunchArgument('prob_hit', default_value='0.7',
                              description='P(occupied | hit) - sensor hit probability'),
        DeclareLaunchArgument('prob_miss', default_value='0.4',
                              description='P(occupied | miss) - ray pass-through probability'),
        DeclareLaunchArgument('prob_prior', default_value='0.5',
                              description='Prior occupancy probability (unknown state)'),
        DeclareLaunchArgument('occupied_threshold', default_value='0.7',
                              description='Occupancy probability threshold'),
        DeclareLaunchArgument('free_threshold', default_value='0.3',
                              description='Free space probability threshold'),
        
        # Visualization
        DeclareLaunchArgument('publish_markers', default_value='true',
                              description='Publish occupancy markers for RViz'),
        DeclareLaunchArgument('publish_point_cloud', default_value='true',
                              description='Publish accumulated point cloud'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz2'),
        
        # Camera intrinsics for information gain
        DeclareLaunchArgument('cam_fx', default_value='500.0'),
        DeclareLaunchArgument('cam_fy', default_value='500.0'),
        DeclareLaunchArgument('cam_cx', default_value='320.0'),
        DeclareLaunchArgument('cam_cy', default_value='240.0'),
        DeclareLaunchArgument('cam_width', default_value='640'),
        DeclareLaunchArgument('cam_height', default_value='480'),
        DeclareLaunchArgument('cam_max_range', default_value='3.0'),
        DeclareLaunchArgument('cam_pixel_stride', default_value='16'),
    ]
    
    # Occupancy map node
    occupancy_map_node = Node(
        package='husky_xarm6_mcr_planning',
        executable='occupancy_map_node',
        name='occupancy_map_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'pointcloud_topic': pointcloud_topic,
            'map_frame': map_frame,
            'resolution': resolution,
            'max_depth': max_depth,
            'downsample_voxel': downsample_voxel,
            
            'prob_hit': prob_hit,
            'prob_miss': prob_miss,
            'prob_prior': prob_prior,
            'occupied_threshold': occupied_threshold,
            'free_threshold': free_threshold,
            
            'publish_markers': publish_markers,
            'publish_point_cloud': publish_pc,
            
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
    
    # RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(use_rviz)
    )
    
    return LaunchDescription(declares + [occupancy_map_node, rviz2])
