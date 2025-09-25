#!/usr/bin/env python3
"""
NED2 Robot Logic Module

This module contains the control logic and interface for the Niryo NED2 robot.
Provides joint position control, trajectory planning, and state management.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose, PoseStamped
import numpy as np
from typing import List, Dict, Optional
import yaml
import os

class NED2Robot(Node):
    """NED2 Robot Controller and Interface"""
    
    def __init__(self, robot_name: str = "ned2"):
        super().__init__(f'{robot_name}_controller')
        
        self.robot_name = robot_name
        self.joint_names = [
            f'joint_{i}' for i in range(1, 7)
        ]
        
        # Load robot specifications from YAML
        self.load_specifications()
        
        # Current robot state
        self.current_joint_positions = np.zeros(6)
        self.current_joint_velocities = np.zeros(6)
        self.target_joint_positions = np.zeros(6)
        
        # ROS2 Publishers
        self.joint_command_pub = self.create_publisher(
            Float64MultiArray,
            f'/{robot_name}/joint_commands',
            10
        )
        
        # ROS2 Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            f'/{robot_name}/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Control timer (100Hz)
        self.control_timer = self.create_timer(0.01, self.control_loop)
        
        self.get_logger().info(f"NED2 Robot Controller initialized for {robot_name}")
    
    def load_specifications(self):
        """Load robot specifications from YAML file"""
        try:
            yaml_path = os.path.join(os.path.dirname(__file__), 'ned2.yaml')
            with open(yaml_path, 'r') as file:
                self.specs = yaml.safe_load(file)
                self.get_logger().info("Robot specifications loaded successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to load specifications: {e}")
            self.specs = {}
    
    def joint_state_callback(self, msg: JointState):
        """Update current joint positions from sensor feedback"""
        if len(msg.position) >= 6:
            self.current_joint_positions = np.array(msg.position[:6])
            self.current_joint_velocities = np.array(msg.velocity[:6]) if msg.velocity else np.zeros(6)
    
    def control_loop(self):
        """Main control loop - position control with basic trajectory planning"""
        # Simple position control
        if np.allclose(self.current_joint_positions, self.target_joint_positions, atol=0.01):
            return  # Already at target
        
        # Calculate command with velocity limiting
        error = self.target_joint_positions - self.current_joint_positions
        max_step = 0.1  # Maximum joint movement per control cycle (radians)
        
        command = np.sign(error) * np.minimum(np.abs(error), max_step)
        commanded_positions = self.current_joint_positions + command
        
        # Publish joint commands
        msg = Float64MultiArray()
        msg.data = commanded_positions.tolist()
        self.joint_command_pub.publish(msg)
    
    def move_to_joint_positions(self, positions: List[float]):
        """Command robot to move to specified joint positions"""
        if len(positions) != 6:
            self.get_logger().error("Joint positions must contain exactly 6 values")
            return False
        
        # Validate joint limits
        if not self.validate_joint_limits(positions):
            self.get_logger().error("Joint positions exceed limits")
            return False
        
        self.target_joint_positions = np.array(positions)
        self.get_logger().info(f"Moving to joint positions: {positions}")
        return True
    
    def validate_joint_limits(self, positions: List[float]) -> bool:
        """Validate that joint positions are within limits"""
        if 'joint_limits' not in self.specs:
            return True  # No limits specified, allow all positions
        
        for i, pos in enumerate(positions):
            joint_name = f'joint_{i+1}'
            if joint_name in self.specs['joint_limits']:
                limits = self.specs['joint_limits'][joint_name]
                min_pos = np.radians(limits['min'])
                max_pos = np.radians(limits['max'])
                
                if pos < min_pos or pos > max_pos:
                    self.get_logger().error(
                        f"Joint {joint_name} position {np.degrees(pos):.1f}° exceeds limits "
                        f"[{limits['min']:.1f}°, {limits['max']:.1f}°]"
                    )
                    return False
        
        return True
    
    def move_to_home_position(self):
        """Move robot to home/default position"""
        home_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        if 'default_position' in self.specs:
            home_positions = [
                np.radians(self.specs['default_position'].get(f'joint_{i}', 0.0))
                for i in range(1, 7)
            ]
        
        self.move_to_joint_positions(home_positions)
    
    def emergency_stop(self):
        """Emergency stop - hold current position"""
        self.target_joint_positions = self.current_joint_positions.copy()
        self.get_logger().warning("Emergency stop activated")
    
    def get_current_pose(self) -> Dict:
        """Get current robot state information"""
        return {
            'joint_positions': self.current_joint_positions.tolist(),
            'joint_velocities': self.current_joint_velocities.tolist(),
            'target_positions': self.target_joint_positions.tolist(),
            'at_target': np.allclose(self.current_joint_positions, self.target_joint_positions, atol=0.01)
        }


def main(args=None):
    """Main entry point for NED2 robot controller"""
    rclpy.init(args=args)
    
    robot = NED2Robot()
    
    try:
        # Move to home position on startup
        robot.move_to_home_position()
        
        rclpy.spin(robot)
    except KeyboardInterrupt:
        robot.get_logger().info("Shutting down NED2 controller")
    finally:
        robot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()