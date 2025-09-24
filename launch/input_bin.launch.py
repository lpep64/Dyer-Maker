#!/usr/bin/env python3
"""
Input Bin Launch File

This launch file spawns an input bin that can periodically spawn
colored blocks (red, green, blue) randomly into the simulation.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "bin_name",
            default_value="input_bin",
            description="Name of the input bin in simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_x",
            default_value="-1.5",
            description="X position to spawn the input bin.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_y",
            default_value="0.0",
            description="Y position to spawn the input bin.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_z",
            default_value="0.0",
            description="Z position to spawn the input bin.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "block_spawn_rate",
            default_value="5.0",
            description="Rate at which to spawn blocks (seconds between spawns).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "auto_spawn",
            default_value="false",
            description="Automatically spawn blocks at regular intervals.",
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
    bin_name = LaunchConfiguration("bin_name")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    block_spawn_rate = LaunchConfiguration("block_spawn_rate")
    auto_spawn = LaunchConfiguration("auto_spawn")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Spawn input bin structure
    spawn_bin_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", bin_name,
            "-x", spawn_x,
            "-y", spawn_y,
            "-z", spawn_z,
            "-string", """
            <?xml version="1.0"?>
            <sdf version="1.6">
                <model name="input_bin">
                    <pose>0 0 0 0 0 0</pose>
                    <static>true</static>
                    
                    <!-- Bin base -->
                    <link name="base_link">
                        <visual name="base_visual">
                            <geometry>
                                <box>
                                    <size>0.5 0.5 0.1</size>
                                </box>
                            </geometry>
                            <material>
                                <ambient>0.2 0.2 0.8 1</ambient>
                                <diffuse>0.2 0.2 0.8 1</diffuse>
                                <specular>0.1 0.1 0.1 1</specular>
                            </material>
                        </visual>
                        <collision name="base_collision">
                            <geometry>
                                <box>
                                    <size>0.5 0.5 0.1</size>
                                </box>
                            </geometry>
                        </collision>
                        <inertial>
                            <mass>10.0</mass>
                            <inertia>
                                <ixx>0.21</ixx>
                                <ixy>0.0</ixy>
                                <ixz>0.0</ixz>
                                <iyy>0.21</iyy>
                                <iyz>0.0</iyz>
                                <izz>0.42</izz>
                            </inertia>
                        </inertial>
                    </link>
                    
                    <!-- Bin walls -->
                    <link name="wall_front">
                        <pose>0.2 0 0.15 0 0 0</pose>
                        <visual name="wall_front_visual">
                            <geometry>
                                <box>
                                    <size>0.05 0.5 0.2</size>
                                </box>
                            </geometry>
                            <material>
                                <ambient>0.2 0.2 0.8 1</ambient>
                                <diffuse>0.2 0.2 0.8 1</diffuse>
                            </material>
                        </visual>
                        <collision name="wall_front_collision">
                            <geometry>
                                <box>
                                    <size>0.05 0.5 0.2</size>
                                </box>
                            </geometry>
                        </collision>
                        <inertial>
                            <mass>2.0</mass>
                            <inertia>
                                <ixx>0.02</ixx>
                                <ixy>0.0</ixy>
                                <ixz>0.0</ixz>
                                <iyy>0.02</iyy>
                                <iyz>0.0</iyz>
                                <izz>0.04</izz>
                            </inertia>
                        </inertial>
                    </link>
                    
                    <link name="wall_left">
                        <pose>0 0.2 0.15 0 0 0</pose>
                        <visual name="wall_left_visual">
                            <geometry>
                                <box>
                                    <size>0.4 0.05 0.2</size>
                                </box>
                            </geometry>
                            <material>
                                <ambient>0.2 0.2 0.8 1</ambient>
                                <diffuse>0.2 0.2 0.8 1</diffuse>
                            </material>
                        </visual>
                        <collision name="wall_left_collision">
                            <geometry>
                                <box>
                                    <size>0.4 0.05 0.2</size>
                                </box>
                            </geometry>
                        </collision>
                        <inertial>
                            <mass>2.0</mass>
                            <inertia>
                                <ixx>0.02</ixx>
                                <ixy>0.0</ixy>
                                <ixz>0.0</ixz>
                                <iyy>0.02</iyy>
                                <iyz>0.0</iyz>
                                <izz>0.04</izz>
                            </inertia>
                        </inertial>
                    </link>
                    
                    <link name="wall_right">
                        <pose>0 -0.2 0.15 0 0 0</pose>
                        <visual name="wall_right_visual">
                            <geometry>
                                <box>
                                    <size>0.4 0.05 0.2</size>
                                </box>
                            </geometry>
                            <material>
                                <ambient>0.2 0.2 0.8 1</ambient>
                                <diffuse>0.2 0.2 0.8 1</diffuse>
                            </material>
                        </visual>
                        <collision name="wall_right_collision">
                            <geometry>
                                <box>
                                    <size>0.4 0.05 0.2</size>
                                </box>
                            </geometry>
                        </collision>
                        <inertial>
                            <mass>2.0</mass>
                            <inertia>
                                <ixx>0.02</ixx>
                                <ixy>0.0</ixy>
                                <ixz>0.0</ixz>
                                <iyy>0.02</iyy>
                                <iyz>0.0</iyz>
                                <izz>0.04</izz>
                            </inertia>
                        </inertial>
                    </link>
                    
                    <!-- Fixed joints to connect walls to base -->
                    <joint name="base_to_wall_front" type="fixed">
                        <parent>base_link</parent>
                        <child>wall_front</child>
                    </joint>
                    
                    <joint name="base_to_wall_left" type="fixed">
                        <parent>base_link</parent>
                        <child>wall_left</child>
                    </joint>
                    
                    <joint name="base_to_wall_right" type="fixed">
                        <parent>base_link</parent>
                        <child>wall_right</child>
                    </joint>
                </model>
            </sdf>
            """
        ],
        output="screen",
    )

    # Block spawner node
    block_spawner_node = Node(
        package="dyer_maker_digital_twin",
        executable="block_spawner",
        name="block_spawner",
        parameters=[{
            "spawn_location_x": spawn_x,
            "spawn_location_y": spawn_y,
            "spawn_location_z": 0.5,  # Above the bin
            "spawn_rate": block_spawn_rate,
            "auto_spawn": auto_spawn,
            "colors": ["red", "green", "blue"],
            "use_sim_time": use_sim_time,
        }],
        output="screen",
    )

    # Delayed spawn to ensure Gazebo is ready
    delayed_spawn_bin = TimerAction(
        period=1.0,
        actions=[spawn_bin_node]
    )

    # Delayed spawn for block spawner to ensure bin is spawned first
    delayed_spawn_blocks = TimerAction(
        period=3.0,
        actions=[block_spawner_node]
    )

    nodes = [
        delayed_spawn_bin,
        delayed_spawn_blocks,
    ]

    return LaunchDescription(declared_arguments + nodes)