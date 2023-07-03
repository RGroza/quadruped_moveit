import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import (OnProcessStart, OnProcessExit)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro
# from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
    )

    pkg_path = os.path.join(get_package_share_directory('quadruped'))
    xacro_file = os.path.join(pkg_path, 'urdf', 'quadruped.urdf.xacro')
    # config_file = os.path.join(
    #     pkg_path, 'config', 'quadruped_controllers.yaml')

    robot_desc_params = {'robot_description': xacro.process_file(
        xacro_file).toxml(), 'use_sim_time': True}

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_desc_params]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', 'quadruped'],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
        output='screen'
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robot_controller", "-c", "/controller_manager"],
        output='screen'
    )

    # load_joint_state_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'joint_state_broadcaster'],
    #     output='screen'
    # )

    # load_robot_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'robot_controller'],
    #     output='screen'
    # )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[robot_controller_spawner],
            )
        ),
        gazebo,
        robot_state_pub_node,
        spawn_entity
    ])
