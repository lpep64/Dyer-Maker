#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
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
    # Declare arguments (similar to your existing launch file)
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="niryo_ned_description",
            description="Description package with robot URDF/xacro files.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file", 
            default_value="ned2/niryo_ned2.urdf.xacro",  # Start with no-mesh version
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true", 
            description="Start Joint State Publisher GUI for manual control.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time from Gazebo.",
        )
    )

    # Initialize Arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Get URDF via xacro (same as your working launch file)
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(description_package),
                    "urdf",
                    description_file,
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Start Gazebo
    gazebo_launch = ExecuteProcess(
        cmd=['gz', 'sim', 'empty.sdf', '-v', '4'],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # Joint State Publisher GUI (for manual control)
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(gui),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Spawn robot in Gazebo
    spawn_entity_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "niryo_ned2",
            "-x", "0.0",
            "-y", "0.0", 
            "-z", "0."
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

    nodes = [
        gazebo_launch,
        robot_state_publisher_node,
        joint_state_publisher_node,
        spawn_entity_node,
        joint_state_bridge,
    ]

    return LaunchDescription(declared_arguments + nodes)
