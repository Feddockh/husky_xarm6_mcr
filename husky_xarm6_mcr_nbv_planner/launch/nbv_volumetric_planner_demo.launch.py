from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch_param_builder import load_xacro
from pathlib import Path
from datetime import datetime
import yaml
import tempfile
import os
import glob

from husky_xarm6_mcr_moveit_config.generate_moveit_controllers_yaml import generate_moveit_controllers_yaml


def load_yaml(path):
    return yaml.safe_load(Path(path).read_text())

def _xacro_param(xacro_path, *args):
    """Build a ParameterValue wrapping a xacro Command.

    Args in *args may include strings and Substitutions (like LaunchConfiguration).
    """
    cmd = ["xacro ", str(xacro_path)] + list(args)
    return ParameterValue(Command(cmd), value_type=str)

def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gazebo = LaunchConfiguration('use_gazebo')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    manipulator_prefix = LaunchConfiguration('manipulator_prefix')
    manipulator_ns = LaunchConfiguration('manipulator_ns')
    platform_prefix = LaunchConfiguration('platform_prefix')
    platform_ns = LaunchConfiguration('platform_ns')
    manipulator_group_name = LaunchConfiguration('manipulator_group_name')
    learn_workspace = LaunchConfiguration('learn_workspace')
    
    # Get package share directory and setup data directory
    package_share_dir = get_package_share_directory('husky_xarm6_mcr_nbv_planner')
    data_dir = os.path.join(package_share_dir, 'data')
    os.makedirs(data_dir, exist_ok=True)
    
    # Determine workspace file path
    learn_workspace_val = learn_workspace.perform(context).lower() == 'true'
    
    if learn_workspace_val:
        # Learning mode: generate timestamped filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        workspace_file_path = os.path.join(data_dir, f'workspace_{timestamp}.bin')
        print(f"[nbv_volumetric_planner_demo.launch.py] Will learn and save workspace to: {workspace_file_path}")
    else:
        # Loading mode: find most recent workspace file
        workspace_files = glob.glob(os.path.join(data_dir, 'workspace_*.bin'))
        
        if workspace_files:
            workspace_files.sort(key=os.path.getmtime, reverse=True)
            workspace_file_path = workspace_files[0]
            print(f"[nbv_volumetric_planner_demo.launch.py] Found {len(workspace_files)} workspace file(s), loading most recent: {os.path.basename(workspace_file_path)}")
        else:
            workspace_file_path = os.path.join(data_dir, 'workspace_latest.bin')
            print(f"[nbv_volumetric_planner_demo.launch.py] WARNING: No workspace files found in {data_dir}")
            print(f"[nbv_volumetric_planner_demo.launch.py] Set learn_workspace:=true to learn a new workspace")

    description_pkg = Path(get_package_share_directory('husky_xarm6_mcr_description'))
    xacro_file = description_pkg / 'urdf' / 'husky_xarm6_mcr.urdf.xacro'
    control_pkg = Path(get_package_share_directory('husky_xarm6_mcr_control'))
    moveit_config_pkg = Path(get_package_share_directory('husky_xarm6_mcr_moveit_config'))

    # Generate the moveit_controllers.yaml file dynamically
    # 1. Generate the config dictionary
    config = generate_moveit_controllers_yaml(
        manipulator_ns=manipulator_ns.perform(context),
        manipulator_prefix=manipulator_prefix.perform(context)
    )
    # 2. Create a temporary file to hold the YAML
    # We use NamedTemporaryFile so it persists long enough for the nodes to read it
    # OS cleans /tmp automatically on reboot, or you can manage cleanup.
    moveit_controllers_yaml_file = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
    yaml.dump(config, moveit_controllers_yaml_file, default_flow_style=False, sort_keys=False)
    moveit_controllers_yaml_file.close()  # Close so other processes can read it
    
    # Load the dynamically generated controllers
    controller_config = yaml.safe_load(Path(moveit_controllers_yaml_file.name).read_text())

    # robot_description from URDF xacro
    robot_description = {'robot_description': _xacro_param(
        description_pkg / 'urdf' / 'husky_xarm6_mcr.urdf.xacro',
        ' use_gazebo:=', use_gazebo,
        ' manipulator_prefix:=', manipulator_prefix,
        ' manipulator_ns:=', manipulator_ns,
        ' platform_prefix:=', platform_prefix,
        ' platform_ns:=', platform_ns
    )}

    # robot_description_semantic from SRDF xacro
    robot_description_semantic = {'robot_description_semantic': _xacro_param(
        description_pkg / 'srdf' / 'husky_xarm6_mcr.srdf.xacro',
        ' manipulator_prefix:=', manipulator_prefix,
        ' manipulator_ns:=', manipulator_ns,
        ' platform_prefix:=', platform_prefix,
        ' platform_ns:=', platform_ns
    )}

    # Load kinematics configuration
    kinematic_yaml = load_yaml(moveit_config_pkg / 'config' / 'kinematics.yaml')
    kinematics_config = {'robot_description_kinematics': kinematic_yaml}

    # Load joint limits configuration
    joint_limits_yaml = load_yaml(moveit_config_pkg / 'config' / 'joint_limits.yaml')
    joint_limits_config = {'robot_description_planning_joint_limits': joint_limits_yaml}

    # Load the planning configs
    ompl_yaml = load_yaml(moveit_config_pkg / 'config' / 'ompl_planning.yaml')
    planner_config = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': ompl_yaml
    }

    # Load the planning scene monitor defaults
    planning_scene_monitor_config = {
        'planning_scene_monitor_options': {
            'name': 'planning_scene_monitor',
            'publish_robot_description': True,
            'publish_robot_description_semantic': True,
            'publish_geometry_updates': True,
            'publish_state_updates': True,
            'publish_transforms_updates': True,
        }
    }

    gt_points_file = os.path.join(
        LaunchConfiguration('gt_points_dir').perform(context),
        LaunchConfiguration('gt_points_file').perform(context)
    )
    nbv_volumetric_planner_demo = Node(
        package='husky_xarm6_mcr_nbv_planner',
        executable='nbv_volumetric_planner_demo',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            robot_description,
            robot_description_semantic,
            kinematics_config,
            joint_limits_config,
            planner_config,
            controller_config,
            planning_scene_monitor_config,
            {'manipulator_group_name': manipulator_group_name},
            {'learn_workspace': learn_workspace},
            {'capture_type': LaunchConfiguration('capture_type')},
            {'manipulation_workspace_file': workspace_file_path},
            {'num_samples': LaunchConfiguration('num_samples')},
            {'visualize': LaunchConfiguration('visualize')},
            {'visualization_topic': LaunchConfiguration('visualization_topic')},
            # NBV Planner Parameters
            {'max_iterations': LaunchConfiguration('max_iterations')},
            {'min_information_gain': LaunchConfiguration('min_information_gain')},
            {'alpha_cost_weight': LaunchConfiguration('alpha_cost_weight')},
            {'num_viewpoints_per_frontier': LaunchConfiguration('num_viewpoints_per_frontier')},
            {'octomap_topic': LaunchConfiguration('octomap_topic')},
            # Camera Parameters
            {'camera_optical_link': LaunchConfiguration('camera_optical_link')},
            {'camera_horizontal_fov': LaunchConfiguration('camera_horizontal_fov')},
            {'camera_vertical_fov': LaunchConfiguration('camera_vertical_fov')},
            {'camera_width': LaunchConfiguration('camera_width')},
            {'camera_height': LaunchConfiguration('camera_height')},
            {'camera_max_range': LaunchConfiguration('camera_max_range')},
            {'ideal_camera_distance': LaunchConfiguration('ideal_camera_distance')},
            {'num_camera_rays': LaunchConfiguration('num_camera_rays')},
            {'map_frame': LaunchConfiguration('map_frame')},
            # User Input Control Parameters
            {'wait_for_user_input': LaunchConfiguration('wait_for_user_input')},
            {'auto_continue_delay': LaunchConfiguration('auto_continue_delay')},
            # Ground Truth Evaluation Parameters
            {'gt_points_file': gt_points_file},
            {'enable_evaluation': LaunchConfiguration('enable_evaluation')},
            {'eval_threshold_radius': LaunchConfiguration('eval_threshold_radius')},
        ],
    )

    return [nbv_volumetric_planner_demo]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_gazebo', default_value='true'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false'),
        DeclareLaunchArgument('manipulator_prefix', default_value='xarm6_'),
        DeclareLaunchArgument('manipulator_ns', default_value='xarm'),
        DeclareLaunchArgument('platform_prefix', default_value='a200_'),
        DeclareLaunchArgument('platform_ns', default_value='husky'),
        DeclareLaunchArgument('manipulator_group_name', default_value='xarm6_manipulator'),
        DeclareLaunchArgument('learn_workspace', default_value='false', 
                            description='True to learn new workspace, False to load existing'),
        DeclareLaunchArgument('capture_type', default_value='triggered',
                            description='Camera capture type: "continuous" or "triggered"'),
        DeclareLaunchArgument('num_samples', default_value='10000000',
                            description='Number of samples for workspace learning'),
        DeclareLaunchArgument('visualize', default_value='true',
                            description='Visualize nbv planner operation in RViz'),
        DeclareLaunchArgument('visualization_topic', default_value='nbv_planner_visualization',
                            description='Topic for workspace visualization markers'),
        # NBV Planner Parameters
        DeclareLaunchArgument('max_iterations', default_value='10',
                            description='Maximum number of NBV planning iterations'),
        DeclareLaunchArgument('min_information_gain', default_value='1.0',
                            description='Minimum information gain threshold for termination'),
        DeclareLaunchArgument('alpha_cost_weight', default_value='0.5',
                            description='Weight for cost in utility function (IG - alpha*cost)'),
        DeclareLaunchArgument('num_viewpoints_per_frontier', default_value='7',
                            description='Number of viewpoint candidates per frontier cluster'),
        DeclareLaunchArgument('octomap_topic', default_value='/octomap_binary',
                            description='Topic for receiving octomap updates'),
        # Camera Parameters
        DeclareLaunchArgument('camera_optical_link', default_value='firefly_left_camera_optical_frame',
                            description='TF frame of the camera optical link'),
        DeclareLaunchArgument('camera_horizontal_fov', default_value='45.0',
                            description='Camera horizontal field of view (degrees, 90 deg default)'),
        DeclareLaunchArgument('camera_vertical_fov', default_value='35.0',
                            description='Camera vertical field of view (degrees, 60 deg default)'),
        DeclareLaunchArgument('camera_width', default_value='448',
                            description='Camera image width (pixels)'),
        DeclareLaunchArgument('camera_height', default_value='224',
                            description='Camera image height (pixels)'),
        DeclareLaunchArgument('camera_max_range', default_value='3.0',
                            description='Camera maximum sensing range (meters)'),
        DeclareLaunchArgument('ideal_camera_distance', default_value='0.5',
                            description='Ideal distance from camera to target surface for information gain computation'),
        DeclareLaunchArgument('num_camera_rays', default_value='25',
                            description='Number of rays for information gain computation'),
        DeclareLaunchArgument('map_frame', default_value='map',
                            description='Fixed frame for visualization markers'),
        DeclareLaunchArgument('wait_for_user_input', default_value='false',
                            description='Wait for user to press Enter at each step (incompatible with ros2 launch)'),
        DeclareLaunchArgument('auto_continue_delay', default_value='3.0',
                            description='Auto-continue delay in seconds when wait_for_user_input is false'),

        # Ground Truth Evaluation Parameters
        DeclareLaunchArgument('gt_points_dir', default_value=PathJoinSubstitution([FindPackageShare('husky_xarm6_mcr_nbv_planner'), 'metrics', 'gt_points']),
            description='Directory containing ground truth points YAML files for semantic evaluation'),
        DeclareLaunchArgument('gt_points_file', default_value='sim_aruco_gt_points.yaml',
                            description='Path to the ground truth points YAML file for semantic evaluation'),
        DeclareLaunchArgument('enable_evaluation', default_value='true',
                            description='Enable semantic octomap evaluation against ground truth'),
        DeclareLaunchArgument('eval_threshold_radius', default_value='0.1',
                            description='Threshold radius (meters) for matching clusters to ground truth points'),
        OpaqueFunction(function=launch_setup),
    ])
