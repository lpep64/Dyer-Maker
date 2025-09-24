#!/usr/bin/env python3
"""
Block Spawner Node

This node periodically spawns colored blocks (red, green, blue) in the simulation
at specified intervals and locations.
"""

import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion
import random
import time
from ros_gz_interfaces.srv import SpawnEntity


class BlockSpawner(Node):
    def __init__(self):
        super().__init__('block_spawner')
        
        # Declare parameters
        self.declare_parameter('spawn_location_x', -1.5)
        self.declare_parameter('spawn_location_y', 0.0) 
        self.declare_parameter('spawn_location_z', 0.5)
        self.declare_parameter('spawn_rate', 5.0)
        self.declare_parameter('auto_spawn', True)
        self.declare_parameter('colors', ['red', 'green', 'blue'])
        
        # Get parameters
        self.spawn_x = self.get_parameter('spawn_location_x').value
        self.spawn_y = self.get_parameter('spawn_location_y').value
        self.spawn_z = self.get_parameter('spawn_location_z').value
        self.spawn_rate = self.get_parameter('spawn_rate').value
        self.auto_spawn = self.get_parameter('auto_spawn').value
        self.colors = self.get_parameter('colors').value
        
        # Block counter for unique naming
        self.block_count = 0
        
        # Create service client for spawning entities
        self.spawn_client = self.create_client(SpawnEntity, '/world/empty/create')
        
        # Wait for service to be available
        self.get_logger().info('Waiting for spawn service...')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        self.get_logger().info(f'Block spawner initialized - spawning every {self.spawn_rate} seconds')
        
        # Create timer for automatic spawning if enabled
        if self.auto_spawn:
            self.spawn_timer = self.create_timer(self.spawn_rate, self.spawn_block_callback)
            self.get_logger().info('Auto-spawning enabled')
        
        # Spawn first block immediately
        self.spawn_random_block()
    
    def spawn_block_callback(self):
        """Timer callback to spawn blocks automatically"""
        self.spawn_random_block()
    
    def spawn_random_block(self):
        """Spawn a random colored block"""
        # Choose random color
        color = random.choice(self.colors)
        self.block_count += 1
        block_name = f"block_{color}_{self.block_count}"
        
        # Add some randomness to position to avoid stacking
        x_offset = random.uniform(-0.1, 0.1)
        y_offset = random.uniform(-0.1, 0.1)
        z_offset = random.uniform(0.0, 0.2)
        
        spawn_x = self.spawn_x + x_offset
        spawn_y = self.spawn_y + y_offset  
        spawn_z = self.spawn_z + z_offset
        
        # Create SDF for the block
        sdf_content = self.create_block_sdf(block_name, color)
        
        # Create spawn request
        request = SpawnEntity.Request()
        request.name = block_name
        request.xml = sdf_content
        request.initial_pose.position.x = spawn_x
        request.initial_pose.position.y = spawn_y
        request.initial_pose.position.z = spawn_z
        request.initial_pose.orientation.x = 0.0
        request.initial_pose.orientation.y = 0.0
        request.initial_pose.orientation.z = 0.0
        request.initial_pose.orientation.w = 1.0
        
        # Send spawn request
        future = self.spawn_client.call_async(request)
        future.add_done_callback(lambda f: self.spawn_callback(f, block_name, color))
    
    def spawn_callback(self, future, block_name, color):
        """Callback for spawn service response"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Successfully spawned {color} block: {block_name}')
            else:
                self.get_logger().warn(f'Failed to spawn block {block_name}: {response.status_message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')
    
    def create_block_sdf(self, name, color):
        """Create SDF content for a colored block"""
        # Color mappings
        color_map = {
            'red': '1 0 0 1',
            'green': '0 1 0 1', 
            'blue': '0 0 1 1'
        }
        
        rgba_color = color_map.get(color, '0.5 0.5 0.5 1')  # Default gray
        
        sdf_content = f'''<?xml version="1.0"?>
<sdf version="1.6">
    <model name="{name}">
        <pose>0 0 0 0 0 0</pose>
        <link name="link">
            <visual name="visual">
                <geometry>
                    <box>
                        <size>0.05 0.05 0.05</size>
                    </box>
                </geometry>
                <material>
                    <ambient>{rgba_color}</ambient>
                    <diffuse>{rgba_color}</diffuse>
                    <specular>0.1 0.1 0.1 1</specular>
                </material>
            </visual>
            <collision name="collision">
                <geometry>
                    <box>
                        <size>0.05 0.05 0.05</size>
                    </box>
                </geometry>
            </collision>
            <inertial>
                <mass>0.1</mass>
                <inertia>
                    <ixx>0.000021</ixx>
                    <ixy>0.0</ixy>
                    <ixz>0.0</ixz>
                    <iyy>0.000021</iyy>
                    <iyz>0.0</iyz>
                    <izz>0.000021</izz>
                </inertia>
            </inertial>
        </link>
    </model>
</sdf>'''
        return sdf_content


def main(args=None):
    rclpy.init(args=args)
    
    block_spawner = BlockSpawner()
    
    try:
        rclpy.spin(block_spawner)
    except KeyboardInterrupt:
        pass
    finally:
        block_spawner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()