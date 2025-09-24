#!/usr/bin/env python3
"""
Conveyor Base Launch File

This launch file spawns a conveyor belt system in Gazebo simulation.
Simple, modular conveyor that can be positioned anywhere.
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
            "conveyor_name",
            default_value="conveyor_belt",
            description="Name of the conveyor in simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_x",
            default_value="1.0",
            description="X position to spawn the conveyor.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_y",
            default_value="0.0",
            description="Y position to spawn the conveyor.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_z",
            default_value="0.0",
            description="Z position to spawn the conveyor.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "length",
            default_value="2.0",
            description="Length of the conveyor belt in meters.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "width",
            default_value="0.3",
            description="Width of the conveyor belt in meters.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "height",
            default_value="0.8",
            description="Height of the conveyor belt in meters.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "speed",
            default_value="0.2",
            description="Belt speed in m/s.",
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
    conveyor_name = LaunchConfiguration("conveyor_name")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    length = LaunchConfiguration("length")
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    speed = LaunchConfiguration("speed")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Spawn conveyor belt as a simple box model
    spawn_conveyor_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", conveyor_name,
            "-x", spawn_x,
            "-y", spawn_y,
            "-z", spawn_z,
            "-string", f"""
            <?xml version="1.0"?>
            <sdf version="1.6">
                <model name="{conveyor_name}">
                    <pose>0 0 0 0 0 0</pose>
                    <static>true</static>
                    <link name="belt_link">
                        <visual name="belt_visual">
                            <geometry>
                                <box>
                                    <size>{length} {width} 0.1</size>
                                </box>
                            </geometry>
                            <material>
                                <ambient>0.3 0.3 0.3 1</ambient>
                                <diffuse>0.3 0.3 0.3 1</diffuse>
                                <specular>0.1 0.1 0.1 1</specular>
                            </material>
                        </visual>
                        <collision name="belt_collision">
                            <geometry>
                                <box>
                                    <size>{length} {width} 0.1</size>
                                </box>
                            </geometry>
                            <surface>
                                <friction>
                                    <ode>
                                        <mu>0.8</mu>
                                        <mu2>0.8</mu2>
                                    </ode>
                                </friction>
                            </surface>
                        </collision>
                        <inertial>
                            <mass>50.0</mass>
                            <inertia>
                                <ixx>4.17</ixx>
                                <ixy>0.0</ixy>
                                <ixz>0.0</ixz>
                                <iyy>4.17</iyy>
                                <iyz>0.0</iyz>
                                <izz>0.83</izz>
                            </inertia>
                        </inertial>
                    </link>
                    
                    <!-- Support structure -->
                    <link name="support_link">
                        <pose>0 0 -{height/2} 0 0 0</pose>
                        <visual name="support_visual">
                            <geometry>
                                <box>
                                    <size>0.1 {width} {height}</size>
                                </box>
                            </geometry>
                            <material>
                                <ambient>0.5 0.5 0.5 1</ambient>
                                <diffuse>0.5 0.5 0.5 1</diffuse>
                            </material>
                        </visual>
                        <collision name="support_collision">
                            <geometry>
                                <box>
                                    <size>0.1 {width} {height}</size>
                                </box>
                            </geometry>
                        </collision>
                        <inertial>
                            <mass>20.0</mass>
                            <inertia>
                                <ixx>1.67</ixx>
                                <ixy>0.0</ixy>
                                <ixz>0.0</ixz>
                                <iyy>1.67</iyy>
                                <iyz>0.0</iyz>
                                <izz>0.33</izz>
                            </inertia>
                        </inertial>
                    </link>
                    
                    <joint name="belt_to_support" type="fixed">
                        <parent>support_link</parent>
                        <child>belt_link</child>
                        <pose>0 0 {height/2 + 0.05} 0 0 0</pose>
                    </joint>
                </model>
            </sdf>
            """
        ],
        output="screen",
    )

    # Conveyor control node (if we implement one later)
    # conveyor_controller = Node(
    #     package="dyer_maker_digital_twin",
    #     executable="conveyor_controller",
    #     name=f"{conveyor_name}_controller",
    #     parameters=[{
    #         "conveyor_name": conveyor_name,
    #         "belt_speed": speed,
    #         "use_sim_time": use_sim_time,
    #     }],
    #     output="screen",
    # )

    # Delayed spawn to ensure Gazebo is ready
    delayed_spawn = TimerAction(
        period=1.0,
        actions=[spawn_conveyor_node]
    )

    nodes = [
        delayed_spawn,
        # conveyor_controller,  # Uncomment when we implement the controller
    ]

    return LaunchDescription(declared_arguments + nodes)