#!/usr/bin/env python3
"""
Single Robot Demo Launch File

This launch file starts a complete digital twin simulation with:
- Gazebo simulation environment
- Single robotic arm (Niryo Ned2)
- Vision system with overhead camera
- Conveyor systems (input/output)
- Pick and place functionality
- System orchestration and coordination
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription,
    GroupAction,
    TimerAction,
    RegisterEventHandler
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, 
    FindExecutable, 
    LaunchConfiguration, 
    PathJoinSubstitution,
    PythonExpression
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package directories
    pkg_dyer_maker = get_package_share_directory('dyer_maker_digital_twin')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Launch arguments
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=PathJoinSubstitution([pkg_dyer_maker, 'worlds', 'factory_testbed.world']),
        description='Path to the Gazebo world file'
    )
    
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='niryo_ned2',
        description='Name of the robot in simulation'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug mode with GUI tools'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz for visualization'
    )
    
    vision_enabled_arg = DeclareLaunchArgument(
        'vision_enabled',
        default_value='true',
        description='Enable vision system'
    )
    
    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='false',
        description='Auto-start the production cycle'
    )

    # Launch configuration variables
    world_file = LaunchConfiguration('world_file')
    robot_name = LaunchConfiguration('robot_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    debug = LaunchConfiguration('debug')
    rviz = LaunchConfiguration('rviz')
    vision_enabled = LaunchConfiguration('vision_enabled')
    auto_start = LaunchConfiguration('auto_start')

    # All declared arguments
    declared_arguments = [
        world_file_arg,
        robot_name_arg,
        use_sim_time_arg,
        debug_arg,
        rviz_arg,
        vision_enabled_arg,
        auto_start_arg
    ]

    # Robot description - Load URDF/XACRO
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([pkg_dyer_maker, 'urdf', 'niryo_ned2.urdf.xacro']),
        ' robot_name:=', robot_name
    ])
    
    robot_description = {'robot_description': robot_description_content}

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'true',
            'pause': 'false'
        }.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )
    
    # Joint State Publisher (for manual control in debug mode)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(
            PythonExpression(["'", debug, "' == 'true'"])
        ),
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', robot_name,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'
        ],
        output='screen'
    )
    
    # Gazebo-ROS2 bridge for joint states
    joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )
    
    # Camera bridge
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/overhead/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/overhead/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ],
        condition=IfCondition(vision_enabled),
        output='screen'
    )
    
    # Vision Node
    vision_node = Node(
        package='dyer_maker_digital_twin',
        executable='vision_node',
        name='vision_node',
        parameters=[{
            'camera_topic': '/camera/overhead/image_raw',
            'config_file': 'config/vision_config.yaml',
            'use_sim_time': use_sim_time,
            'publish_debug_images': True
        }],
        condition=IfCondition(vision_enabled),
        output='screen'
    )
    
    # Sensor Coordinator Node
    sensor_coordinator = Node(
        package='dyer_maker_digital_twin',
        executable='sensor_coordinator_node',
        name='sensor_coordinator',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Pick and Place Node
    pick_place_node = Node(
        package='dyer_maker_digital_twin',
        executable='pick_place_node',
        name='pick_place_controller',
        parameters=[{
            'robot_name': robot_name,
            'config_file': 'config/robot_config.yaml',
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    
    # System Orchestrator Node
    system_orchestrator = Node(
        package='dyer_maker_digital_twin',
        executable='system_orchestrator_node',
        name='system_orchestrator',
        parameters=[{
            'auto_start': auto_start,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    
    # Conveyor Nodes
    input_conveyor_node = Node(
        package='dyer_maker_digital_twin',
        executable='conveyor_node',
        name='input_conveyor',
        parameters=[{
            'conveyor_id': 'input_conveyor',
            'config_file': 'config/conveyor_config.yaml',
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    
    output_conveyor_node = Node(
        package='dyer_maker_digital_twin',
        executable='conveyor_node',
        name='output_conveyor',
        parameters=[{
            'conveyor_id': 'output_conveyor',
            'config_file': 'config/conveyor_config.yaml',
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    
    # RViz for visualization
    rviz_config = PathJoinSubstitution([
        pkg_dyer_maker,
        'rviz',
        'single_robot_demo.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
        output='screen'
    )
    
    # Group nodes that should start after Gazebo
    gazebo_dependent_nodes = GroupAction(actions=[
        spawn_entity,
        joint_state_bridge,
        camera_bridge,
    ])
    
    # Group application nodes that should start after robot is spawned
    application_nodes = GroupAction(actions=[
        vision_node,
        sensor_coordinator,
        pick_place_node,
        system_orchestrator,
        input_conveyor_node,
        output_conveyor_node,
    ])
    
    # Start sequence with delays
    delayed_gazebo_nodes = TimerAction(
        period=3.0,  # Wait for Gazebo to start
        actions=[gazebo_dependent_nodes]
    )
    
    delayed_application_nodes = TimerAction(
        period=6.0,  # Wait for robot to spawn
        actions=[application_nodes]
    )
    
    delayed_rviz = TimerAction(
        period=8.0,  # Start RViz after everything else
        actions=[rviz_node]
    )
    
    # Event handlers for coordinated startup
    gazebo_started_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=gazebo,
            on_start=[delayed_gazebo_nodes]
        )
    )
    
    robot_spawned_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=spawn_entity,
            on_start=[delayed_application_nodes]
        )
    )
    
    # Create launch description
    nodes = [
        gazebo,
        robot_state_publisher,
        joint_state_publisher_gui,
        gazebo_started_handler,
        robot_spawned_handler,
        delayed_rviz,
    ]
    
    return LaunchDescription(declared_arguments + nodes)