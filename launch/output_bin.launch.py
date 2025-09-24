#!/usr/bin/env python3
"""
Output Bin Launch File

This launch file spawns an output bin for collecting processed blocks.
The bin can have multiple compartments for different colored blocks.
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
            default_value="output_bin",
            description="Name of the output bin in simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_x",
            default_value="1.5",
            description="X position to spawn the output bin.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_y",
            default_value="0.0",
            description="Y position to spawn the output bin.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_z",
            default_value="0.0",
            description="Z position to spawn the output bin.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "compartments",
            default_value="3",
            description="Number of compartments in the output bin (1-3).",
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
    compartments = LaunchConfiguration("compartments")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Spawn output bin structure (3-compartment version)
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
                <model name="output_bin">
                    <pose>0 0 0 0 0 0</pose>
                    <static>true</static>
                    
                    <!-- Main base -->
                    <link name="base_link">
                        <visual name="base_visual">
                            <geometry>
                                <box>
                                    <size>0.8 1.2 0.1</size>
                                </box>
                            </geometry>
                            <material>
                                <ambient>0.8 0.8 0.2 1</ambient>
                                <diffuse>0.8 0.8 0.2 1</diffuse>
                                <specular>0.1 0.1 0.1 1</specular>
                            </material>
                        </visual>
                        <collision name="base_collision">
                            <geometry>
                                <box>
                                    <size>0.8 1.2 0.1</size>
                                </box>
                            </geometry>
                        </collision>
                        <inertial>
                            <mass>15.0</mass>
                            <inertia>
                                <ixx>1.8</ixx>
                                <ixy>0.0</ixy>
                                <ixz>0.0</ixz>
                                <iyy>0.8</iyy>
                                <iyz>0.0</iyz>
                                <izz>2.44</izz>
                            </inertia>
                        </inertial>
                    </link>
                    
                    <!-- Red compartment -->
                    <link name="red_compartment">
                        <pose>0 -0.35 0.15 0 0 0</pose>
                        <visual name="red_compartment_visual">
                            <geometry>
                                <box>
                                    <size>0.7 0.3 0.2</size>
                                </box>
                            </geometry>
                            <material>
                                <ambient>0.8 0.2 0.2 0.7</ambient>
                                <diffuse>0.8 0.2 0.2 0.7</diffuse>
                            </material>
                        </visual>
                        <!-- Compartment walls -->
                        <collision name="red_wall_front">
                            <pose>0.3 0 0 0 0 0</pose>
                            <geometry>
                                <box>
                                    <size>0.05 0.3 0.2</size>
                                </box>
                            </geometry>
                        </collision>
                        <collision name="red_wall_back">
                            <pose>-0.3 0 0 0 0 0</pose>
                            <geometry>
                                <box>
                                    <size>0.05 0.3 0.2</size>
                                </box>
                            </geometry>
                        </collision>
                        <collision name="red_wall_left">
                            <pose>0 0.125 0 0 0 0</pose>
                            <geometry>
                                <box>
                                    <size>0.6 0.05 0.2</size>
                                </box>
                            </geometry>
                        </collision>
                        <collision name="red_wall_right">
                            <pose>0 -0.125 0 0 0 0</pose>
                            <geometry>
                                <box>
                                    <size>0.6 0.05 0.2</size>
                                </box>
                            </geometry>
                        </collision>
                        <inertial>
                            <mass>3.0</mass>
                            <inertia>
                                <ixx>0.03</ixx>
                                <ixy>0.0</ixy>
                                <ixz>0.0</ixz>
                                <iyy>0.12</iyy>
                                <iyz>0.0</iyz>
                                <izz>0.12</izz>
                            </inertia>
                        </inertial>
                    </link>
                    
                    <!-- Green compartment -->
                    <link name="green_compartment">
                        <pose>0 0.0 0.15 0 0 0</pose>
                        <visual name="green_compartment_visual">
                            <geometry>
                                <box>
                                    <size>0.7 0.3 0.2</size>
                                </box>
                            </geometry>
                            <material>
                                <ambient>0.2 0.8 0.2 0.7</ambient>
                                <diffuse>0.2 0.8 0.2 0.7</diffuse>
                            </material>
                        </visual>
                        <collision name="green_wall_front">
                            <pose>0.3 0 0 0 0 0</pose>
                            <geometry>
                                <box>
                                    <size>0.05 0.3 0.2</size>
                                </box>
                            </geometry>
                        </collision>
                        <collision name="green_wall_back">
                            <pose>-0.3 0 0 0 0 0</pose>
                            <geometry>
                                <box>
                                    <size>0.05 0.3 0.2</size>
                                </box>
                            </geometry>
                        </collision>
                        <collision name="green_wall_left">
                            <pose>0 0.125 0 0 0 0</pose>
                            <geometry>
                                <box>
                                    <size>0.6 0.05 0.2</size>
                                </box>
                            </geometry>
                        </collision>
                        <collision name="green_wall_right">
                            <pose>0 -0.125 0 0 0 0</pose>
                            <geometry>
                                <box>
                                    <size>0.6 0.05 0.2</size>
                                </box>
                            </geometry>
                        </collision>
                        <inertial>
                            <mass>3.0</mass>
                            <inertia>
                                <ixx>0.03</ixx>
                                <ixy>0.0</ixy>
                                <ixz>0.0</ixz>
                                <iyy>0.12</iyy>
                                <iyz>0.0</iyz>
                                <izz>0.12</izz>
                            </inertia>
                        </inertial>
                    </link>
                    
                    <!-- Blue compartment -->
                    <link name="blue_compartment">
                        <pose>0 0.35 0.15 0 0 0</pose>
                        <visual name="blue_compartment_visual">
                            <geometry>
                                <box>
                                    <size>0.7 0.3 0.2</size>
                                </box>
                            </geometry>
                            <material>
                                <ambient>0.2 0.2 0.8 0.7</ambient>
                                <diffuse>0.2 0.2 0.8 0.7</diffuse>
                            </material>
                        </visual>
                        <collision name="blue_wall_front">
                            <pose>0.3 0 0 0 0 0</pose>
                            <geometry>
                                <box>
                                    <size>0.05 0.3 0.2</size>
                                </box>
                            </geometry>
                        </collision>
                        <collision name="blue_wall_back">
                            <pose>-0.3 0 0 0 0 0</pose>
                            <geometry>
                                <box>
                                    <size>0.05 0.3 0.2</size>
                                </box>
                            </geometry>
                        </collision>
                        <collision name="blue_wall_left">
                            <pose>0 0.125 0 0 0 0</pose>
                            <geometry>
                                <box>
                                    <size>0.6 0.05 0.2</size>
                                </box>
                            </geometry>
                        </collision>
                        <collision name="blue_wall_right">
                            <pose>0 -0.125 0 0 0 0</pose>
                            <geometry>
                                <box>
                                    <size>0.6 0.05 0.2</size>
                                </box>
                            </geometry>
                        </collision>
                        <inertial>
                            <mass>3.0</mass>
                            <inertia>
                                <ixx>0.03</ixx>
                                <ixy>0.0</ixy>
                                <ixz>0.0</ixz>
                                <iyy>0.12</iyy>
                                <iyz>0.0</iyz>
                                <izz>0.12</izz>
                            </inertia>
                        </inertial>
                    </link>
                    
                    <!-- Fixed joints to connect compartments to base -->
                    <joint name="base_to_red" type="fixed">
                        <parent>base_link</parent>
                        <child>red_compartment</child>
                    </joint>
                    
                    <joint name="base_to_green" type="fixed">
                        <parent>base_link</parent>
                        <child>green_compartment</child>
                    </joint>
                    
                    <joint name="base_to_blue" type="fixed">
                        <parent>base_link</parent>
                        <child>blue_compartment</child>
                    </joint>
                </model>
            </sdf>
            """
        ],
        output="screen",
    )

    # Block collector node (would need to be implemented)
    # block_collector_node = Node(
    #     package="dyer_maker_digital_twin",
    #     executable="block_collector",
    #     name="block_collector",
    #     parameters=[{
    #         "bin_location_x": spawn_x,
    #         "bin_location_y": spawn_y,
    #         "bin_location_z": spawn_z,
    #         "compartments": compartments,
    #         "use_sim_time": use_sim_time,
    #     }],
    #     output="screen",
    # )

    # Delayed spawn to ensure Gazebo is ready
    delayed_spawn = TimerAction(
        period=1.0,
        actions=[spawn_bin_node]
    )

    nodes = [
        delayed_spawn,
        # block_collector_node,  # Uncomment when we implement the collector
    ]

    return LaunchDescription(declared_arguments + nodes)