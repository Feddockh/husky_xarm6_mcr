"""
Launch file to save the current octomap to a file
Subscribes to octomap topic and saves it as .ot or .bt file
Saves to share/husky_xarm6_mcr_occupancy_map/data/ directory
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from datetime import datetime
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package share directory (install/share/package_name)
    package_share_dir = get_package_share_directory('husky_xarm6_mcr_occupancy_map')
    data_dir = os.path.join(package_share_dir, 'data')
    
    # Create data directory if it doesn't exist
    os.makedirs(data_dir, exist_ok=True)
    
    # Generate default filename with timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    default_filename = f'octomap_{timestamp}.bt'
    
    print(f"[save_octomap.launch.py] Will save to: {data_dir}")
    print(f"[save_octomap.launch.py] Default filename: {default_filename}")
    
    # Declare arguments
    octomap_topic_arg = DeclareLaunchArgument(
        'octomap_topic',
        default_value='/octomap_binary',
        description='Topic to subscribe for octomap messages'
    )
    
    filename_arg = DeclareLaunchArgument(
        'filename',
        default_value=default_filename,
        description='Filename for the octomap (will be saved in data/ directory)'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Node to save octomap using custom saver
    save_octomap_node = Node(
        package='husky_xarm6_mcr_occupancy_map',
        executable='octomap_saver',
        name='octomap_saver',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'output_file': [data_dir, '/', LaunchConfiguration('filename')],
            'octomap_topic': LaunchConfiguration('octomap_topic')
        }]
    )

    return LaunchDescription([
        octomap_topic_arg,
        filename_arg,
        use_sim_time_arg,
        save_octomap_node
    ])
