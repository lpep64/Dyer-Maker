#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node


def generate_launch_description():
    # Use direct path to URDF file
    robot_description_path = "/home/empise/documents/Dyer-Maker/models/robots/ned2/ned2.urdf.xacro"

    gazebo_node = ExecuteProcess(
        cmd=["gz", "sim", "empty.sdf", "-r"],
        output="screen",
    )

    robot_description_content = Command([
        FindExecutable(name="xacro"),
        " ",
        robot_description_path,
    ])

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description_content}],
        output="screen",
    )

    spawn_ned2 = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "niryo_ned2",
            "-topic", "robot_description",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.0",
        ],
        output="screen",
    )

    # Simple joint position command publisher to keep joints at home position
    joint_commander = ExecuteProcess(
        cmd=["/home/empise/documents/Dyer-Maker/scripts/hold_joints.sh"],
        output="screen",
    )

    delayed_robot_state = TimerAction(period=1.0, actions=[robot_state_publisher_node])
    delayed_spawn_ned2 = TimerAction(period=3.0, actions=[spawn_ned2])
    # Removed joint_commander to allow manual control

    return LaunchDescription([
        gazebo_node,
        delayed_robot_state,
        delayed_spawn_ned2,
    ])
