#!/usr/bin/env python3
import os
import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Use the XArm helpers (same ones used by the server launches)
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from uf_ros_lib.uf_robot_utils import generate_ros2_control_params_temp_file


def _real_moveit_config(context):
    # --- match real bringup defaults/args ---
    robot_ip = LaunchConfiguration('robot_ip')
    report_type = LaunchConfiguration('report_type', default='normal')
    baud_checkset = LaunchConfiguration('baud_checkset', default=True)
    default_gripper_baud = LaunchConfiguration('default_gripper_baud', default=2000000)

    dof = LaunchConfiguration('dof', default=7)
    robot_type = LaunchConfiguration('robot_type', default='xarm')
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=True)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    model1300 = LaunchConfiguration('model1300', default=False)
    robot_sn = LaunchConfiguration('robot_sn', default='')
    attach_to = LaunchConfiguration('attach_to', default='world')
    attach_xyz = LaunchConfiguration('attach_xyz', default='"0 0 0"')
    attach_rpy = LaunchConfiguration('attach_rpy', default='"0 0 0"')
    mesh_suffix = LaunchConfiguration('mesh_suffix', default='stl')
    kinematics_suffix = LaunchConfiguration('kinematics_suffix', default='')

    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)
    add_realsense_d435i = LaunchConfiguration('add_realsense_d435i', default=False)
    add_d435i_links = LaunchConfiguration('add_d435i_links', default=True)
    add_other_geometry = LaunchConfiguration('add_other_geometry', default=False)
    geometry_type = LaunchConfiguration('geometry_type', default='box')
    geometry_mass = LaunchConfiguration('geometry_mass', default=0.1)
    geometry_height = LaunchConfiguration('geometry_height', default=0.1)
    geometry_radius = LaunchConfiguration('geometry_radius', default=0.1)
    geometry_length = LaunchConfiguration('geometry_length', default=0.1)
    geometry_width = LaunchConfiguration('geometry_width', default=0.1)
    geometry_mesh_filename = LaunchConfiguration('geometry_mesh_filename', default='')
    geometry_mesh_origin_xyz = LaunchConfiguration('geometry_mesh_origin_xyz', default='"0 0 0"')
    geometry_mesh_origin_rpy = LaunchConfiguration('geometry_mesh_origin_rpy', default='"0 0 0"')
    geometry_mesh_tcp_xyz = LaunchConfiguration('geometry_mesh_tcp_xyz', default='"0 0 0"')
    geometry_mesh_tcp_rpy = LaunchConfiguration('geometry_mesh_tcp_rpy', default='"0 0 0"')

    ros_namespace = LaunchConfiguration('ros_namespace', default='').perform(context)

    ros2_control_plugin = 'uf_robot_hardware/UFRobotSystemHardware'
    controllers_name = 'controllers'
    xarm_type = '{}{}'.format(
        robot_type.perform(context),
        dof.perform(context) if robot_type.perform(context) in ('xarm', 'lite') else ''
    )

    ros2_control_params = generate_ros2_control_params_temp_file(
        os.path.join(get_package_share_directory('xarm_controller'), 'config', f'{xarm_type}_controllers.yaml'),
        prefix=LaunchConfiguration('prefix').perform(context),
        add_gripper=add_gripper.perform(context) in ('True', 'true'),
        add_bio_gripper=add_bio_gripper.perform(context) in ('True', 'true'),
        ros_namespace=ros_namespace,
        robot_type=robot_type.perform(context)
    )

    return MoveItConfigsBuilder(
        context=context,
        controllers_name=controllers_name,
        robot_ip=robot_ip,
        report_type=report_type,
        baud_checkset=baud_checkset,
        default_gripper_baud=default_gripper_baud,
        dof=dof,
        robot_type=robot_type,
        prefix=LaunchConfiguration('prefix'),
        hw_ns=LaunchConfiguration('hw_ns'),
        limited=limited,
        effort_control=effort_control,
        velocity_control=velocity_control,
        model1300=model1300,
        robot_sn=robot_sn,
        attach_to=attach_to,
        attach_xyz=attach_xyz,
        attach_rpy=attach_rpy,
        mesh_suffix=mesh_suffix,
        kinematics_suffix=kinematics_suffix,
        ros2_control_plugin=ros2_control_plugin,
        ros2_control_params=ros2_control_params,
        add_gripper=add_gripper,
        add_vacuum_gripper=add_vacuum_gripper,
        add_bio_gripper=add_bio_gripper,
        add_realsense_d435i=add_realsense_d435i,
        add_d435i_links=add_d435i_links,
        add_other_geometry=add_other_geometry,
        geometry_type=geometry_type,
        geometry_mass=geometry_mass,
        geometry_height=geometry_height,
        geometry_radius=geometry_radius,
        geometry_length=geometry_length,
        geometry_width=geometry_width,
        geometry_mesh_filename=geometry_mesh_filename,
        geometry_mesh_origin_xyz=geometry_mesh_origin_xyz,
        geometry_mesh_origin_rpy=geometry_mesh_origin_rpy,
        geometry_mesh_tcp_xyz=geometry_mesh_tcp_xyz,
        geometry_mesh_tcp_rpy=geometry_mesh_tcp_rpy,
    ).to_moveit_configs()


def _fake_moveit_config(context):
    # --- match fake bringup defaults/args ---
    dof = LaunchConfiguration('dof', default=7)
    robot_type = LaunchConfiguration('robot_type', default='xarm')
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=True)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    model1300 = LaunchConfiguration('model1300', default=False)
    robot_sn = LaunchConfiguration('robot_sn', default='')
    attach_to = LaunchConfiguration('attach_to', default='world')
    attach_xyz = LaunchConfiguration('attach_xyz', default='"0 0 0"')
    attach_rpy = LaunchConfiguration('attach_rpy', default='"0 0 0"')
    mesh_suffix = LaunchConfiguration('mesh_suffix', default='stl')
    kinematics_suffix = LaunchConfiguration('kinematics_suffix', default='')

    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)
    add_realsense_d435i = LaunchConfiguration('add_realsense_d435i', default=False)
    add_d435i_links = LaunchConfiguration('add_d435i_links', default=True)
    add_other_geometry = LaunchConfiguration('add_other_geometry', default=False)
    geometry_type = LaunchConfiguration('geometry_type', default='box')
    geometry_mass = LaunchConfiguration('geometry_mass', default=0.1)
    geometry_height = LaunchConfiguration('geometry_height', default=0.1)
    geometry_radius = LaunchConfiguration('geometry_radius', default=0.1)
    geometry_length = LaunchConfiguration('geometry_length', default=0.1)
    geometry_width = LaunchConfiguration('geometry_width', default=0.1)
    geometry_mesh_filename = LaunchConfiguration('geometry_mesh_filename', default='')
    geometry_mesh_origin_xyz = LaunchConfiguration('geometry_mesh_origin_xyz', default='"0 0 0"')
    geometry_mesh_origin_rpy = LaunchConfiguration('geometry_mesh_origin_rpy', default='"0 0 0"')
    geometry_mesh_tcp_xyz = LaunchConfiguration('geometry_mesh_tcp_xyz', default='"0 0 0"')
    geometry_mesh_tcp_rpy = LaunchConfiguration('geometry_mesh_tcp_rpy', default='"0 0 0"')

    ros_namespace = LaunchConfiguration('ros_namespace', default='').perform(context)

    ros2_control_plugin = 'uf_robot_hardware/UFRobotFakeSystemHardware'
    controllers_name = 'fake_controllers'
    xarm_type = '{}{}'.format(
        robot_type.perform(context),
        dof.perform(context) if robot_type.perform(context) in ('xarm', 'lite') else ''
    )

    ros2_control_params = generate_ros2_control_params_temp_file(
        os.path.join(get_package_share_directory('xarm_controller'), 'config', f'{xarm_type}_controllers.yaml'),
        prefix=LaunchConfiguration('prefix').perform(context),
        add_gripper=add_gripper.perform(context) in ('True', 'true'),
        add_bio_gripper=add_bio_gripper.perform(context) in ('True', 'true'),
        ros_namespace=ros_namespace,
        robot_type=robot_type.perform(context)
    )

    return MoveItConfigsBuilder(
        context=context,
        controllers_name=controllers_name,
        dof=dof,
        robot_type=robot_type,
        prefix=LaunchConfiguration('prefix'),
        hw_ns=LaunchConfiguration('hw_ns'),
        limited=limited,
        effort_control=effort_control,
        velocity_control=velocity_control,
        model1300=model1300,
        robot_sn=robot_sn,
        attach_to=LaunchConfiguration('attach_to'),
        attach_xyz=LaunchConfiguration('attach_xyz'),
        attach_rpy=LaunchConfiguration('attach_rpy'),
        mesh_suffix=LaunchConfiguration('mesh_suffix'),
        kinematics_suffix=LaunchConfiguration('kinematics_suffix'),
        ros2_control_plugin=ros2_control_plugin,
        ros2_control_params=ros2_control_params,
        add_gripper=add_gripper,
        add_vacuum_gripper=add_vacuum_gripper,
        add_bio_gripper=add_bio_gripper,
        add_realsense_d435i=add_realsense_d435i,
        add_d435i_links=add_d435i_links,
        add_other_geometry=add_other_geometry,
        geometry_type=geometry_type,
        geometry_mass=geometry_mass,
        geometry_height=geometry_height,
        geometry_radius=geometry_radius,
        geometry_length=geometry_length,
        geometry_width=geometry_width,
        geometry_mesh_filename=geometry_mesh_filename,
        geometry_mesh_origin_xyz=geometry_mesh_origin_xyz,
        geometry_mesh_origin_rpy=geometry_mesh_origin_rpy,
        geometry_mesh_tcp_xyz=geometry_mesh_tcp_xyz,
        geometry_mesh_tcp_rpy=geometry_mesh_tcp_rpy,
    ).to_moveit_configs()


def _launch_client(context, *args, **kwargs):
    fake = LaunchConfiguration('fake').perform(context)
    hw_ns = LaunchConfiguration('hw_ns').perform(context)
    moveit_config = _fake_moveit_config(context) if fake in ('true', 'True', '1') else _real_moveit_config(context)

    remaps = []
    if fake.lower() in ('false', '0', 'no'):
        # Real hardware: subscribe to /xarm/joint_states (or /<hw_ns>/joint_states)
        remaps.append(('joint_states', f'/{hw_ns}/joint_states'))
        # Fake mode usually publishes on /joint_states already, so no remap needed.

    # This is the executable you want to run (e.g., plan_to_xyz)
    node = Node(
        package='husky_xarm6_mcr_bringup',
        executable=LaunchConfiguration('exe'),
        output='screen',
        parameters=[
            # Minimum required:
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            # Helpful extras to match the server side:
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.planning_pipelines,
            moveit_config.trajectory_execution,
            moveit_config.planning_scene_monitor,
            moveit_config.sensors_3d,
            {"use_sim_time": True}, # This is for the current time (for some this is needed for the joint states to be read correctly)
        ],
        remappings=remaps,
    )
    return [node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('exe'),
        DeclareLaunchArgument('fake', default_value='true'),
        DeclareLaunchArgument('robot_ip', default_value=''),
        DeclareLaunchArgument('report_type', default_value='normal'),
        DeclareLaunchArgument('baud_checkset', default_value='true'),
        DeclareLaunchArgument('default_gripper_baud', default_value='2000000'),
        DeclareLaunchArgument('dof', default_value='6'),
        DeclareLaunchArgument('robot_type', default_value='xarm'),
        DeclareLaunchArgument('prefix', default_value=''),
        DeclareLaunchArgument('hw_ns', default_value='xarm'),
        DeclareLaunchArgument('limited', default_value='true'),
        DeclareLaunchArgument('effort_control', default_value='false'),
        DeclareLaunchArgument('velocity_control', default_value='false'),
        DeclareLaunchArgument('model1300', default_value='false'),
        DeclareLaunchArgument('robot_sn', default_value=''),
        DeclareLaunchArgument('attach_to', default_value='world'),
        DeclareLaunchArgument('attach_xyz', default_value='"0 0 0"'),
        DeclareLaunchArgument('attach_rpy', default_value='"0 0 0"'),
        DeclareLaunchArgument('mesh_suffix', default_value='stl'),
        DeclareLaunchArgument('kinematics_suffix', default_value=''),
        DeclareLaunchArgument('add_gripper', default_value='false'),
        DeclareLaunchArgument('add_vacuum_gripper', default_value='false'),
        DeclareLaunchArgument('add_bio_gripper', default_value='false'),
        DeclareLaunchArgument('add_realsense_d435i', default_value='false'),
        DeclareLaunchArgument('add_d435i_links', default_value='true'),
        DeclareLaunchArgument('add_other_geometry', default_value='false'),
        DeclareLaunchArgument('geometry_type', default_value='box'),
        DeclareLaunchArgument('geometry_mass', default_value='0.1'),
        DeclareLaunchArgument('geometry_height', default_value='0.1'),
        DeclareLaunchArgument('geometry_radius', default_value='0.1'),
        DeclareLaunchArgument('geometry_length', default_value='0.1'),
        DeclareLaunchArgument('geometry_width', default_value='0.1'),
        DeclareLaunchArgument('geometry_mesh_filename', default_value=''),
        DeclareLaunchArgument('geometry_mesh_origin_xyz', default_value='"0 0 0"'),
        DeclareLaunchArgument('geometry_mesh_origin_rpy', default_value='"0 0 0"'),
        DeclareLaunchArgument('geometry_mesh_tcp_xyz', default_value='"0 0 0"'),
        DeclareLaunchArgument('geometry_mesh_tcp_rpy', default_value='"0 0 0"'),
        DeclareLaunchArgument('ros_namespace', default_value=''),
        OpaqueFunction(function=_launch_client),
    ])

# ros2 launch husky_xarm6_mcr_bringup run.launch.py exe:=plan_to_xyz fake:=true dof:=6 robot_type:=xarm
# ros2 launch husky_xarm6_mcr_bringup run.launch.py exe:=plan_to_xyz fake:=false robot_ip:=192.168.1.205 dof:=6 robot_type:=xarm