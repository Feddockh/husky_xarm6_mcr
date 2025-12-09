from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'joy_dev', 
            default_value='/dev/input/js0', 
            description='Path to joystick device'
        ),
        
        # 1. Driver Node: Reads hardware events
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': LaunchConfiguration('joy_dev'),
                'deadzone': 0.1,
                'autorepeat_rate': 20.0, # Publish continuously so robot doesn't stop
            }],
            output='screen'
        ),

        # 2. Teleop Node: Converts to Twist
        Node(
            package='husky_xarm6_mcr_control',
            executable='xbox_teleop.py',
            name='xbox_teleop',
            parameters=[{
                'linear_axis': 4,   # Right Stick Vertical
                'angular_axis': 3,  # Right Stick Horizontal
                'scale_linear': 0.5,
                'scale_angular': 1.0,
                'enable_button': 0, # 'A' Button to drive
                'boost_button': 5,  # 'RB' for speed boost
            }],
            output='screen'
        )
    ])