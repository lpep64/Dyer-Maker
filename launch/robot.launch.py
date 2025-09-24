#!/usr/bin/env python3
"""
Robot Launch File - Niryo Ned2 in Gazebo

This launch file spawns a single Niryo Ned2 robot in Gazebo simulation.
Based on niryo_gazebo_simple_launch.py but cleaned up and focused.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_name",
            default_value="niryo_ned2",
            description="Name of the robot in simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_x",
            default_value="0.0",
            description="X position to spawn the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_y",
            default_value="0.0",
            description="Y position to spawn the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_z",
            default_value="0.0",
            description="Z position to spawn the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time from Gazebo.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Start Joint State Publisher GUI for manual control.",
        )
    )

    # Initialize Arguments
    robot_name = LaunchConfiguration("robot_name")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")

    # Get URDF via xacro - using our local URDF files
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("dyer_maker_digital_twin"),
                    "urdf",
                    "niryo_ned2_proper.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )

    # Joint State Publisher GUI (for manual control)
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(gui),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Spawn robot in Gazebo (delayed to ensure Gazebo is ready)
    spawn_entity_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", robot_name,
            "-x", spawn_x,
            "-y", spawn_y,
            "-z", spawn_z,
        ],
        output="screen",
    )

    # Bridge joint states from Gazebo to ROS
    joint_state_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model"
        ],
        output="screen",
    )

    # Delayed spawn to ensure robot description is published
    delayed_spawn = TimerAction(
        period=2.0,
        actions=[spawn_entity_node]
    )

    # Delayed bridge to ensure robot is spawned
    delayed_bridge = TimerAction(
        period=3.0,
        actions=[joint_state_bridge]
    )

    nodes = [
        robot_state_publisher_node,
        joint_state_publisher_node,
        delayed_spawn,
        delayed_bridge,
    ]

    return LaunchDescription(declared_arguments + nodes)