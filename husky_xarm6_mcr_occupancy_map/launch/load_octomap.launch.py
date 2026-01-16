"""
Launch file to load and publish a saved octomap
Reads octomap from .ot or .bt file and continuously publishes it
This allows NBV planner development without running full Gazebo simulation
Loads from share/husky_xarm6_mcr_occupancy_map/data/ directory
By default, loads the most recent timestamped octomap file
"""

import os
import glob
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Get the package share directory (install/share/package_name)
    package_share_dir = get_package_share_directory('husky_xarm6_mcr_occupancy_map')
    data_dir = os.path.join(package_share_dir, 'data')
    
    # Create data directory if it doesn't exist
    os.makedirs(data_dir, exist_ok=True)
    
    # Find the most recent octomap file in the data directory
    octomap_files = glob.glob(os.path.join(data_dir, 'octomap_*.bt'))
    octomap_files.extend(glob.glob(os.path.join(data_dir, 'octomap_*.ot')))
    
    if octomap_files:
        # Sort by modification time, most recent first
        octomap_files.sort(key=os.path.getmtime, reverse=True)
        default_map_path = octomap_files[0]
        print(f"[load_octomap.launch.py] Found {len(octomap_files)} octomap file(s), defaulting to most recent: {os.path.basename(default_map_path)}")
    else:
        # Fallback: provide a helpful error message path
        default_map_path = os.path.join(data_dir, 'octomap_YYYYMMDD_HHMMSS.bt')
        print(f"[load_octomap.launch.py] WARNING: No octomap files found in {data_dir}")
        print(f"[load_octomap.launch.py] Please save an octomap first using save_octomap.launch.py")
        print(f"[load_octomap.launch.py] Or specify a file with: octomap_file:=/path/to/your/file.bt")

    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time (false for standalone use)'
    )

    octomap_file_arg = DeclareLaunchArgument(
        'octomap_file',
        default_value=default_map_path,
        description='Path to the octomap file to load (.ot or .bt). Must exist!'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/octomap_binary',
        description='Topic to publish the loaded octomap'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='map',
        description='Frame ID for the octomap'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='1.0',
        description='Rate (Hz) to republish the octomap'
    )
    
    publish_free_arg = DeclareLaunchArgument(
        'publish_free',
        default_value='true',
        description='Whether to visualize free voxels (can be slow for large maps)'
    )
    
    enable_visualization_arg = DeclareLaunchArgument(
        'enable_visualization',
        default_value='true',
        description='Whether to enable visualization markers'
    )
    
    visualization_topic_arg = DeclareLaunchArgument(
        'visualization_topic',
        default_value='occupancy_map_markers',
        description='Topic to publish visualization markers on'
    )
    
    visualization_rate_arg = DeclareLaunchArgument(
        'visualization_rate',
        default_value='1.0',
        description='Rate (Hz) to republish visualization markers'
    )
    
    show_bbox_arg = DeclareLaunchArgument(
        'show_bounding_box',
        default_value='true',
        description='Whether to visualize bounding box'
    )
    
    bbx_min_x_arg = DeclareLaunchArgument(
        'bbx_min_x',
        default_value='-1.0',
        description='Bounding box minimum X'
    )
    
    bbx_min_y_arg = DeclareLaunchArgument(
        'bbx_min_y',
        default_value='-2.0',
        description='Bounding box minimum Y'
    )
    
    bbx_min_z_arg = DeclareLaunchArgument(
        'bbx_min_z',
        default_value='0.0',
        description='Bounding box minimum Z'
    )
    
    bbx_max_x_arg = DeclareLaunchArgument(
        'bbx_max_x',
        default_value='1.0',
        description='Bounding box maximum X'
    )
    
    bbx_max_y_arg = DeclareLaunchArgument(
        'bbx_max_y',
        default_value='-0.5',
        description='Bounding box maximum Y'
    )
    
    bbx_max_z_arg = DeclareLaunchArgument(
        'bbx_max_z',
        default_value='2.0',
        description='Bounding box maximum Z'
    )

    # Node to load and publish octomap using custom loader
    load_octomap_node = Node(
        package='husky_xarm6_mcr_occupancy_map',
        executable='octomap_loader',
        name='octomap_loader',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'input_file': LaunchConfiguration('octomap_file'),
            'output_topic': LaunchConfiguration('output_topic'),
            'frame_id': LaunchConfiguration('frame_id'),
            'publish_rate_hz': LaunchConfiguration('publish_rate'),
            'publish_free_voxels': LaunchConfiguration('publish_free'),
            'enable_visualization': LaunchConfiguration('enable_visualization'),
            'visualization_topic': LaunchConfiguration('visualization_topic'),
            'visualization_rate_hz': LaunchConfiguration('visualization_rate'),
            'show_bounding_box': LaunchConfiguration('show_bounding_box'),
            'bbx_min_x': LaunchConfiguration('bbx_min_x'),
            'bbx_min_y': LaunchConfiguration('bbx_min_y'),
            'bbx_min_z': LaunchConfiguration('bbx_min_z'),
            'bbx_max_x': LaunchConfiguration('bbx_max_x'),
            'bbx_max_y': LaunchConfiguration('bbx_max_y'),
            'bbx_max_z': LaunchConfiguration('bbx_max_z')
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        octomap_file_arg,
        output_topic_arg,
        frame_id_arg,
        publish_rate_arg,
        publish_free_arg,
        enable_visualization_arg,
        visualization_topic_arg,
        visualization_rate_arg,
        show_bbox_arg,
        bbx_min_x_arg,
        bbx_min_y_arg,
        bbx_min_z_arg,
        bbx_max_x_arg,
        bbx_max_y_arg,
        bbx_max_z_arg,
        load_octomap_node
    ])
