from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from pathlib import Path
import yaml


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

    desc_pkg = Path(get_package_share_directory('husky_xarm6_mcr_description'))
    cfg_pkg = Path(get_package_share_directory('husky_xarm6_mcr_moveit_config'))

    manip_prefix = LaunchConfiguration('manipulator_prefix')
    plat_prefix = LaunchConfiguration('platform_prefix')

    # robot_description from URDF xacro
    robot_description = {'robot_description': _xacro_param(
        desc_pkg / 'urdf' / 'husky_xarm6_mcr.urdf.xacro',
        ' sim:=true',
        ' manipulator_prefix:=', manip_prefix,
        ' platform_prefix:=', plat_prefix
    )}

    # robot_description_semantic from SRDF xacro
    robot_description_semantic = {'robot_description_semantic': _xacro_param(
        desc_pkg / 'srdf' / 'husky_xarm6_mcr.srdf.xacro',
        ' manipulator_prefix:=', manip_prefix,
        ' platform_prefix:=', plat_prefix
    )}

    # Load kinematics configuration
    kinematic_yaml = load_yaml(cfg_pkg / 'config' / 'kinematics.yaml')
    kinematics_config = {'robot_description_kinematics': kinematic_yaml}

    # Load the planning configs
    ompl_yaml = load_yaml(cfg_pkg / 'config' / 'ompl_planning.yaml')
    planner_config = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': ompl_yaml
    }
    
    # Load the controllers
    controller_config = load_yaml(cfg_pkg / 'config' / 'moveit_controllers.yaml')

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

    rviz_config = str(cfg_pkg / 'config' / 'moveit.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[
            {'use_sim_time': True},
            robot_description,
            robot_description_semantic,
            kinematics_config,
            planner_config,
            controller_config,
            planning_scene_monitor_config,
        ],
    )

    return [rviz_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('manipulator_prefix', default_value='xarm6_'),
        DeclareLaunchArgument('platform_prefix', default_value='a200_'),
        OpaqueFunction(function=launch_setup),
    ])