import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    actions = []

    # ------------------------------------------------------------
    # 1) Launch Gazebo world (your existing launch)
    # ------------------------------------------------------------
    pkg_gz = get_package_share_directory('husky_xarm6_mcr_gz')
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_gz, 'launch', 'gazebo.launch.py')),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'generate_markers': LaunchConfiguration('generate_markers'),
                'num_markers': LaunchConfiguration('num_markers'),
                'marker_dict': LaunchConfiguration('marker_dict'),
                'marker_size': LaunchConfiguration('marker_size'),
            }.items()
        )
    )

    # ------------------------------------------------------------
    # 2) Spawn the camera rig as a standalone Gazebo entity
    #    This uses your multi_camera_rig.urdf.xacro
    # ------------------------------------------------------------
    rig_xacro = PathJoinSubstitution([
        FindPackageShare('multi_camera_rig_description'),
        'urdf',
        'multi_camera_rig_description.urdf.xacro'
    ])

    robot_description = ParameterValue(
        Command(['xacro ', rig_xacro, ' ', 'use_gazebo:=true']),
        value_type=str
    )

    # robot_state_publisher provides /robot_description for ros_gz_sim/create
    actions.append(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='camera_probe_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
            }],
        )
    )

    # Spawn into Gazebo as entity camera_probe
    actions.append(
        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-name', LaunchConfiguration('camera_entity'),
                '-topic', 'robot_description',
                '-x', LaunchConfiguration('spawn_x'),
                '-y', LaunchConfiguration('spawn_y'),
                '-z', LaunchConfiguration('spawn_z'),
            ],
        )
    )

    # ------------------------------------------------------------
    # 3) Fake trigger node (gates raw -> triggered topics)
    #    NOTE: update topic names here if your Gazebo firefly topics differ.
    # ------------------------------------------------------------
    # ---- build real python strings ----
    image_sub_topics = [
        '/firefly_left/image_raw',
        '/firefly_right/image_raw',
    ]
    image_pub_topics = [
        '/firefly_left/image_raw/triggered',
        '/firefly_right/image_raw/triggered',
    ]
    info_sub_topics = [
        '/firefly_left/camera_info',
        '/firefly_right/camera_info',
    ]
    info_pub_topics = [
        '/firefly_left/camera_info/triggered',
        '/firefly_right/camera_info/triggered',
    ]

    trigger_node = Node(
        package='multi_camera_rig_trigger',
        executable='trigger_node_fake',
        name='trigger_node',
        output='screen',
        parameters=[{
            'flash_duration_ms': int(LaunchConfiguration('trigger_flash_duration_ms').perform(context)),
            'frame_rate_hz': int(LaunchConfiguration('trigger_frame_rate_hz').perform(context)),
            'auto_connect': True,
            'auto_start': False,

            # IMPORTANT: these must be real python lists of strings
            'image_sub_topics': image_sub_topics,
            'image_pub_topics': image_pub_topics,
            'info_sub_topics': info_sub_topics,
            'info_pub_topics': info_pub_topics,
        }]
    )
    actions.append(trigger_node)

    # ------------------------------------------------------------
    # 4) Rectify nodes (subscribe to triggered topics)
    # ------------------------------------------------------------
    output_width = int(LaunchConfiguration('output_width').perform(context))
    output_height = int(LaunchConfiguration('output_height').perform(context))

    for cam_name, in_img, in_info, out_rect in [
        ('firefly_left',  LaunchConfiguration('left_triggered_image'),  LaunchConfiguration('left_triggered_info'),  LaunchConfiguration('left_rect_image')),
        ('firefly_right', LaunchConfiguration('right_triggered_image'), LaunchConfiguration('right_triggered_info'), LaunchConfiguration('right_rect_image')),
    ]:
        actions.append(
            Node(
                package='multi_camera_rig_reconstruction',
                executable='stereo_rectify_scale_node',
                name=f'{cam_name}_rectify_scale',
                output='screen',
                parameters=[{
                    'in_image_topic': in_img,
                    'in_info_topic': in_info,
                    'out_rect_image_topic': out_rect,
                    'out_rect_info_topic': f'/{cam_name}/camera_info_rect',
                    'publish_scaled': False,

                    'output_width': output_width,
                    'output_height': output_height,
                    'interpolation': 'linear',

                    # QoS (match your pipeline)
                    'sub_qos.reliability': 'reliable',
                    'sub_qos.durability': 'volatile',
                    'sub_qos.history': 'keep_last',
                    'sub_qos.depth': 5,
                    'pub_qos.reliability': 'best_effort',
                    'pub_qos.durability': 'volatile',
                    'pub_qos.history': 'keep_last',
                    'pub_qos.depth': 5,
                }]
            )
        )

    # ------------------------------------------------------------
    # 5) Image saver nodes (let these do all disk saving)
    #    Save rectified images (not scaled by default — pick what you want).
    # ------------------------------------------------------------
    save_root = LaunchConfiguration('save_directory').perform(context)
    if save_root:
        for cam_name, rect_topic in [
            ('firefly_left', LaunchConfiguration('left_rect_image').perform(context)),
            ('firefly_right', LaunchConfiguration('right_rect_image').perform(context)),
        ]:
            actions.append(
                Node(
                    package='multi_camera_rig_bringup',  # <-- where image_saver_node is
                    executable='image_saver_node',
                    name=f'{cam_name}_image_saver',
                    output='screen',
                    parameters=[{
                        'image_topic': rect_topic,
                        'save_directory': os.path.join(save_root, cam_name),
                        'image_prefix': cam_name,
                        'image_format': LaunchConfiguration('image_format').perform(context),
                        'qos_depth': 5,
                        'qos_reliability': 'best_effort',
                        'qos_durability': 'volatile',
                        'qos_history': 'keep_last',
                    }]
                )
            )

    # ------------------------------------------------------------
    # 6) Capture mover node (teleport + trigger + metadata)
    # ------------------------------------------------------------
    actions.append(
        Node(
            package='husky_xarm6_mcr_gz',                    # <-- adjust to where you install it
            executable='gazebo_viewpoint_capture_node.py',   # <-- entrypoint name
            name='gazebo_viewpoint_capture_node',
            output='screen',
            parameters=[{
                'gt_yaml': LaunchConfiguration('gt_yaml').perform(context),
                'output_dir': save_root,  # metadata will be saved here
                'camera_entity': LaunchConfiguration('camera_entity').perform(context),
                'world_name': LaunchConfiguration('world_name').perform(context),
                'trigger_service': LaunchConfiguration('trigger_service').perform(context),

                # Wait topic (use left rect or triggered; just used as an ack)
                'ack_image_topic': LaunchConfiguration('left_rect_image').perform(context),

                # Where to look for newest saved file (left camera)
                'left_save_dir': os.path.join(save_root, 'firefly_left'),

                # sampling
                'num_views_per_point': int(LaunchConfiguration('num_views_per_point').perform(context)),
                'min_radius': float(LaunchConfiguration('min_radius').perform(context)),
                'max_radius': float(LaunchConfiguration('max_radius').perform(context)),
                'seed': int(LaunchConfiguration('seed').perform(context)),
            }]
        )
    )

    return actions


def generate_launch_description():
    return LaunchDescription([
        # Gazebo world args
        DeclareLaunchArgument('world', default_value='apple_orchard'),
        DeclareLaunchArgument('generate_markers', default_value='true'),
        DeclareLaunchArgument('num_markers', default_value='20'),
        DeclareLaunchArgument('marker_dict', default_value='DICT_4X4_50'),
        DeclareLaunchArgument('marker_size', default_value='0.05'),

        # Spawn camera rig entity
        DeclareLaunchArgument('camera_entity', default_value='camera_probe'),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_z', default_value='1.2'),

        # World name for /world/<name>/set_entity_pose (often "default")
        DeclareLaunchArgument('world_name', default_value='default'),

        # Trigger node executable location
        DeclareLaunchArgument('trigger_pkg', default_value='multi_camera_rig_trigger'),  # <-- change if needed
        DeclareLaunchArgument('trigger_exec', default_value='trigger_node_fake'),       # <-- change if needed
        DeclareLaunchArgument('trigger_flash_duration_ms', default_value='200'),
        DeclareLaunchArgument('trigger_frame_rate_hz', default_value='10'),
        DeclareLaunchArgument('trigger_service', default_value='/trigger/send_trigger'),

        # Firefly topic wiring (CHANGE THESE if your sim topics differ)
        DeclareLaunchArgument('left_raw_image', default_value='/firefly_left/image_raw'),
        DeclareLaunchArgument('right_raw_image', default_value='/firefly_right/image_raw'),
        DeclareLaunchArgument('left_raw_info', default_value='/firefly_left/camera_info'),
        DeclareLaunchArgument('right_raw_info', default_value='/firefly_right/camera_info'),

        DeclareLaunchArgument('left_triggered_image', default_value='/firefly_left/image_raw/triggered'),
        DeclareLaunchArgument('right_triggered_image', default_value='/firefly_right/image_raw/triggered'),
        DeclareLaunchArgument('left_triggered_info', default_value='/firefly_left/camera_info/triggered'),
        DeclareLaunchArgument('right_triggered_info', default_value='/firefly_right/camera_info/triggered'),

        # Rectified output topics
        DeclareLaunchArgument('left_rect_image', default_value='/firefly_left/image_rect'),
        DeclareLaunchArgument('right_rect_image', default_value='/firefly_right/image_rect'),

        # Rectify/scale parameters
        DeclareLaunchArgument('output_width', default_value='448'),
        DeclareLaunchArgument('output_height', default_value='224'),

        # Dataset params
        DeclareLaunchArgument('gt_yaml', default_value=''),
        DeclareLaunchArgument('save_directory', default_value='/home/hayden/tmp/saved_images'),
        DeclareLaunchArgument('image_format', default_value='png'),

        DeclareLaunchArgument('num_views_per_point', default_value='10'),
        DeclareLaunchArgument('min_radius', default_value='0.15'),
        DeclareLaunchArgument('max_radius', default_value='0.60'),
        DeclareLaunchArgument('seed', default_value='0'),

        OpaqueFunction(function=launch_setup),
    ])