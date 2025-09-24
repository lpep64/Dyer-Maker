#!/usr/bin/env python3
"""
Master Launch File - Dyer Maker Digital Twin

This master launch file orchestrates the modular launch files to create
different demo scenarios. It can launch various combinations of:
- Gazebo simulation environment
- Robot
- Conveyor system
- Input bin
- Output bin

Usage examples:
ros2 launch dyer_maker_digital_twin master.launch.py scenario:=basic
ros2 launch dyer_maker_digital_twin master.launch.py scenario:=full robot_x:=0.5
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package directory
    pkg_share = FindPackageShare("dyer_maker_digital_twin")
    
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "scenario",
            default_value="basic",
            description="Demo scenario: basic, conveyor, full",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "world_file",
            default_value="empty.sdf",
            description="Gazebo world file to load",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_x",
            default_value="0.0",
            description="Robot X position",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_y",
            default_value="0.0",
            description="Robot Y position",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Enable robot joint state publisher GUI",
        )
    )

    # Initialize Arguments
    scenario = LaunchConfiguration("scenario")
    world_file = LaunchConfiguration("world_file")
    robot_x = LaunchConfiguration("robot_x")
    robot_y = LaunchConfiguration("robot_y")
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")

    # Start Gazebo
    gazebo_launch = ExecuteProcess(
        cmd=["gz", "sim", world_file, "-v", "4"],
        output="screen"
    )

    # Robot launch
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, "launch", "robot.launch.py"])
        ]),
        launch_arguments={
            "spawn_x": robot_x,
            "spawn_y": robot_y,
            "use_sim_time": use_sim_time,
            "gui": gui,
        }.items(),
    )

    # Conveyor launch (for conveyor and full scenarios)
    conveyor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, "launch", "conveyor.launch.py"])
        ]),
        launch_arguments={
            "spawn_x": "1.0",
            "spawn_y": "0.0",
            "use_sim_time": use_sim_time,
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", scenario, "' == 'conveyor' or '", scenario, "' == 'full'"])
        ),
    )

    # Input bin launch (for full scenario)
    input_bin_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, "launch", "input_bin.launch.py"])
        ]),
        launch_arguments={
            "spawn_x": "-1.5",
            "spawn_y": "0.0",
            "use_sim_time": use_sim_time,
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", scenario, "' == 'full'"])
        ),
    )

    # Output bin launch (for full scenario)
    output_bin_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, "launch", "output_bin.launch.py"])
        ]),
        launch_arguments={
            "spawn_x": "2.5",
            "spawn_y": "0.0",
            "use_sim_time": use_sim_time,
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", scenario, "' == 'full'"])
        ),
    )

    # Delayed launches to ensure proper startup sequence
    delayed_robot = TimerAction(
        period=3.0,
        actions=[robot_launch]
    )

    delayed_conveyor = TimerAction(
        period=5.0,
        actions=[conveyor_launch]
    )

    delayed_input_bin = TimerAction(
        period=6.0,
        actions=[input_bin_launch]
    )

    delayed_output_bin = TimerAction(
        period=7.0,
        actions=[output_bin_launch]
    )

    return LaunchDescription(
        declared_arguments + [
            gazebo_launch,
            delayed_robot,
            delayed_conveyor,
            delayed_input_bin,
            delayed_output_bin,
        ]
    )