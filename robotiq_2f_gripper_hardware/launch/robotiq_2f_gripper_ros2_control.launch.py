#!/usr/bin/env python3
"""
ROS 2 Control Launch File for Robotiq 2F-140 Gripper

This launch file demonstrates using the Robotiq gripper with the ros2_control
hardware interface instead of the standalone action server.

Usage:
  # Real hardware:
  ros2 launch robotiq_2f_gripper_hardware robotiq_2f_gripper_ros2_control.launch.py

  # Fake hardware (for testing):
  ros2 launch robotiq_2f_gripper_hardware robotiq_2f_gripper_ros2_control.launch.py use_fake_hardware:=true

  # With custom serial port:
  ros2 launch robotiq_2f_gripper_hardware robotiq_2f_gripper_ros2_control.launch.py serial_port:=/dev/ttyUSB1

Command the gripper:
  ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.7, max_effort: 50.0}}"

Author: Auto-generated for Robotiq 2F Gripper ROS 2 Control Integration
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB1',
            description='Serial port for the gripper'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'baudrate',
            default_value='115200',
            description='Baudrate for serial communication'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'slave_address',
            default_value='9',
            description='Modbus slave address of the gripper'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='Use fake hardware for testing (no real gripper needed)'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Use Gazebo simulation'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'controller_config_file',
            default_value='gripper_controllers.yaml',
            description='Controller configuration YAML file name'
        )
    )

    # Initialize Arguments
    serial_port = LaunchConfiguration('serial_port')
    baudrate = LaunchConfiguration('baudrate')
    slave_address = LaunchConfiguration('slave_address')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    use_sim = LaunchConfiguration('use_sim')
    controller_config_file = LaunchConfiguration('controller_config_file')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            'xacro ',
            PathJoinSubstitution([
                FindPackageShare('robotiq_2f_gripper_hardware'),
                'urdf',
                'robotiq_2f_gripper_standalone.urdf.xacro'
            ]),
            ' serial_port:=', serial_port,
            ' baudrate:=', baudrate,
            ' slave_address:=', slave_address,
            ' use_fake_hardware:=', use_fake_hardware,
            ' use_sim:=', use_sim,
        ]
    )
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Controller configuration file path
    controller_config_path = PathJoinSubstitution(
        [
            FindPackageShare('robotiq_2f_gripper_hardware'),
            'config',
            controller_config_file,
        ]
    )

    # Controller Manager (ros2_control node)
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            controller_config_path
        ],
        output='screen',
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Spawn Joint State Broadcaster
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Spawn Gripper Controller
    spawn_gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Delay spawning controllers after controller_manager starts
    spawn_controllers_callback = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                spawn_joint_state_broadcaster,
                spawn_gripper_controller,
            ]
        )
    )

    nodes = [
        control_node,
        robot_state_publisher_node,
        spawn_controllers_callback,
    ]

    return LaunchDescription(declared_arguments + nodes)
