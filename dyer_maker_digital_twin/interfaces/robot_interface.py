#!/usr/bin/env python3
"""
Robot Interface Module for Dyer-Maker Digital Twin

This module provides a modular interface for robot control and motion planning,
allowing easy integration with different robot types and control systems.

Author: Dyer-Maker Team
License: MIT
"""

from abc import ABC, abstractmethod
from typing import Dict, List, Tuple, Optional
import numpy as np
from dataclasses import dataclass
from enum import Enum
import time

class RobotState(Enum):
    """Robot operational states"""
    IDLE = "idle"
    MOVING = "moving"
    PICKING = "picking"
    PLACING = "placing"
    ERROR = "error"
    EMERGENCY_STOP = "emergency_stop"

class GripperState(Enum):
    """Gripper states"""
    OPEN = "open"
    CLOSED = "closed"
    GRIPPING = "gripping"
    ERROR = "error"

@dataclass
class RobotPose:
    """Robot pose data structure"""
    position: Tuple[float, float, float]  # (x, y, z)
    orientation: Tuple[float, float, float, float]  # Quaternion (x, y, z, w)
    timestamp: float

@dataclass
class JointState:
    """Joint state information"""
    joint_names: List[str]
    positions: List[float]
    velocities: List[float]
    efforts: List[float]
    timestamp: float

@dataclass
class PickPlaceCommand:
    """Command for pick and place operations"""
    pick_pose: RobotPose
    place_pose: RobotPose
    approach_offset: float = 0.05  # meters above target
    retreat_offset: float = 0.05   # meters above target
    pick_speed: float = 0.1        # m/s
    place_speed: float = 0.1       # m/s
    gripper_force: float = 50.0    # Newtons

class RobotInterface(ABC):
    """Abstract base class for robot control interfaces"""
    
    @abstractmethod
    def initialize(self, config: Dict) -> bool:
        """
        Initialize the robot interface
        
        Args:
            config: Configuration parameters
            
        Returns:
            bool: True if initialization successful
        """
        pass
    
    @abstractmethod
    def get_current_pose(self) -> RobotPose:
        """Get current robot end-effector pose"""
        pass
    
    @abstractmethod
    def get_joint_states(self) -> JointState:
        """Get current joint states"""
        pass
    
    @abstractmethod
    def move_to_pose(self, target_pose: RobotPose, speed: float = 0.1) -> bool:
        """
        Move robot to target pose
        
        Args:
            target_pose: Target end-effector pose
            speed: Movement speed in m/s
            
        Returns:
            bool: True if movement successful
        """
        pass
    
    @abstractmethod
    def execute_pick_place(self, command: PickPlaceCommand) -> bool:
        """
        Execute a pick and place operation
        
        Args:
            command: Pick and place command
            
        Returns:
            bool: True if operation successful
        """
        pass
    
    @abstractmethod
    def control_gripper(self, command: str, force: float = 50.0) -> bool:
        """
        Control gripper
        
        Args:
            command: 'open', 'close', or 'grip'
            force: Gripping force in Newtons
            
        Returns:
            bool: True if command successful
        """
        pass
    
    @abstractmethod
    def get_robot_state(self) -> RobotState:
        """Get current robot operational state"""
        pass
    
    @abstractmethod
    def emergency_stop(self) -> bool:
        """Execute emergency stop"""
        pass
    
    @abstractmethod
    def get_capabilities(self) -> Dict:
        """Get robot capabilities and limits"""
        pass

class NiryoRobotInterface(RobotInterface):
    """
    Niryo robot interface implementation
    
    This implementation provides control interface for Niryo Ned robots
    using ROS2 and MoveIt2 integration.
    """
    
    def __init__(self, robot_name: str = "niryo_ned2"):
        """Initialize Niryo robot interface"""
        self.robot_name = robot_name
        self.current_state = RobotState.IDLE
        self.gripper_state = GripperState.OPEN
        
        # Robot capabilities and limits
        self.capabilities = {
            'max_reach': 0.55,  # meters
            'max_payload': 0.3,  # kg
            'joint_limits': {
                'joint_1': (-175, 175),  # degrees
                'joint_2': (-80, 80),
                'joint_3': (-90, 90),
                'joint_4': (-110, 110),
                'joint_5': (-170, 170),
                'joint_6': (-180, 180)
            },
            'max_speed': 0.5,  # m/s
            'position_accuracy': 0.001,  # meters
            'gripper_force_range': (10, 100)  # Newtons
        }
        
        # Current robot state
        self.current_pose = RobotPose(
            position=(0.3, 0.0, 0.2),
            orientation=(0.0, 0.0, 0.0, 1.0),
            timestamp=time.time()
        )
        
        # Safety and configuration parameters
        self.safety_limits = {
            'workspace_bounds': {
                'x_min': -0.6, 'x_max': 0.6,
                'y_min': -0.6, 'y_max': 0.6,
                'z_min': -0.1, 'z_max': 0.6
            },
            'max_acceleration': 1.0,  # m/s²
            'collision_detection': True
        }
        
        # Performance metrics
        self.metrics = {
            'successful_operations': 0,
            'failed_operations': 0,
            'total_operation_time': 0.0,
            'average_cycle_time': 0.0
        }
    
    def initialize(self, config: Dict) -> bool:
        """Initialize the Niryo robot interface"""
        try:
            # Update configuration parameters
            if 'robot_name' in config:
                self.robot_name = config['robot_name']
            
            if 'safety_limits' in config:
                self.safety_limits.update(config['safety_limits'])
            
            if 'capabilities' in config:
                self.capabilities.update(config['capabilities'])
            
            # Initialize MoveIt2 interface (placeholder)
            self._initialize_moveit()
            
            # Initialize gripper interface (placeholder)
            self._initialize_gripper()
            
            # Perform homing sequence
            success = self._home_robot()
            
            if success:
                self.current_state = RobotState.IDLE
                return True
            else:
                self.current_state = RobotState.ERROR
                return False
                
        except Exception as e:
            print(f"Robot initialization error: {e}")
            self.current_state = RobotState.ERROR
            return False
    
    def _initialize_moveit(self):
        """Initialize MoveIt2 motion planning interface"""
        # Placeholder for MoveIt2 initialization
        # In real implementation, this would initialize:
        # - MoveGroupInterface
        # - PlanningSceneInterface
        # - Robot model and kinematic solver
        print(f"Initializing MoveIt2 for {self.robot_name}")
    
    def _initialize_gripper(self):
        """Initialize gripper control interface"""
        # Placeholder for gripper initialization
        # In real implementation, this would initialize:
        # - Gripper action server client
        # - Force feedback interface
        # - Gripper state monitoring
        print(f"Initializing gripper for {self.robot_name}")
    
    def _home_robot(self) -> bool:
        """Move robot to home position"""
        home_pose = RobotPose(
            position=(0.3, 0.0, 0.25),
            orientation=(0.0, 0.0, 0.0, 1.0),
            timestamp=time.time()
        )
        
        return self.move_to_pose(home_pose, speed=0.1)
    
    def get_current_pose(self) -> RobotPose:
        """Get current robot end-effector pose"""
        # Update timestamp
        self.current_pose.timestamp = time.time()
        return self.current_pose
    
    def get_joint_states(self) -> JointState:
        """Get current joint states"""
        # Placeholder implementation - in real system would get from ROS topics
        return JointState(
            joint_names=['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'],
            positions=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Home position
            velocities=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            efforts=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            timestamp=time.time()
        )
    
    def move_to_pose(self, target_pose: RobotPose, speed: float = 0.1) -> bool:
        """Move robot to target pose"""
        try:
            # Validate target pose
            if not self._validate_pose(target_pose):
                print("Invalid target pose - outside workspace or safety limits")
                return False
            
            # Validate speed
            if speed > self.capabilities['max_speed']:
                print(f"Speed {speed} exceeds maximum {self.capabilities['max_speed']}")
                speed = self.capabilities['max_speed']
            
            self.current_state = RobotState.MOVING
            
            # Simulate motion planning and execution
            start_time = time.time()
            
            # In real implementation, this would:
            # 1. Plan trajectory using MoveIt2
            # 2. Check for collisions
            # 3. Execute trajectory
            # 4. Monitor execution and handle errors
            
            # Simulate motion time based on distance
            distance = self._calculate_pose_distance(self.current_pose, target_pose)
            motion_time = distance / speed
            
            # Simulate motion (in real system, this would be async)
            time.sleep(min(motion_time, 0.1))  # Cap simulation time
            
            # Update current pose
            self.current_pose = target_pose
            self.current_pose.timestamp = time.time()
            
            # Update metrics
            operation_time = time.time() - start_time
            self.metrics['total_operation_time'] += operation_time
            self.metrics['successful_operations'] += 1
            
            self._update_average_cycle_time()
            
            self.current_state = RobotState.IDLE
            return True
            
        except Exception as e:
            print(f"Motion execution error: {e}")
            self.current_state = RobotState.ERROR
            self.metrics['failed_operations'] += 1
            return False
    
    def execute_pick_place(self, command: PickPlaceCommand) -> bool:
        """Execute a pick and place operation"""
        try:
            start_time = time.time()
            
            # Step 1: Move to approach position for pick
            approach_pick = RobotPose(
                position=(command.pick_pose.position[0],
                         command.pick_pose.position[1],
                         command.pick_pose.position[2] + command.approach_offset),
                orientation=command.pick_pose.orientation,
                timestamp=time.time()
            )
            
            if not self.move_to_pose(approach_pick, command.pick_speed):
                print("Failed to move to pick approach position")
                return False
            
            # Step 2: Move down to pick position
            if not self.move_to_pose(command.pick_pose, command.pick_speed * 0.5):
                print("Failed to move to pick position")
                return False
            
            # Step 3: Close gripper to pick up part
            self.current_state = RobotState.PICKING
            if not self.control_gripper("close", command.gripper_force):
                print("Failed to close gripper")
                return False
            
            # Step 4: Retreat from pick position
            if not self.move_to_pose(approach_pick, command.pick_speed * 0.5):
                print("Failed to retreat from pick position")
                return False
            
            # Step 5: Move to approach position for place
            approach_place = RobotPose(
                position=(command.place_pose.position[0],
                         command.place_pose.position[1],
                         command.place_pose.position[2] + command.retreat_offset),
                orientation=command.place_pose.orientation,
                timestamp=time.time()
            )
            
            if not self.move_to_pose(approach_place, command.place_speed):
                print("Failed to move to place approach position")
                return False
            
            # Step 6: Move down to place position
            if not self.move_to_pose(command.place_pose, command.place_speed * 0.5):
                print("Failed to move to place position")
                return False
            
            # Step 7: Open gripper to release part
            self.current_state = RobotState.PLACING
            if not self.control_gripper("open"):
                print("Failed to open gripper")
                return False
            
            # Step 8: Retreat from place position
            if not self.move_to_pose(approach_place, command.place_speed * 0.5):
                print("Failed to retreat from place position")
                return False
            
            # Update metrics
            operation_time = time.time() - start_time
            self.metrics['total_operation_time'] += operation_time
            self.metrics['successful_operations'] += 1
            self._update_average_cycle_time()
            
            self.current_state = RobotState.IDLE
            return True
            
        except Exception as e:
            print(f"Pick and place operation error: {e}")
            self.current_state = RobotState.ERROR
            self.metrics['failed_operations'] += 1
            return False
    
    def control_gripper(self, command: str, force: float = 50.0) -> bool:
        """Control gripper"""
        try:
            # Validate force
            min_force, max_force = self.capabilities['gripper_force_range']
            if force < min_force or force > max_force:
                print(f"Gripper force {force}N outside range [{min_force}, {max_force}]")
                return False
            
            # Execute gripper command
            if command.lower() == "open":
                self.gripper_state = GripperState.OPEN
                # Simulate gripper opening time
                time.sleep(0.1)
                return True
                
            elif command.lower() == "close" or command.lower() == "grip":
                self.gripper_state = GripperState.CLOSED
                # Simulate gripper closing time
                time.sleep(0.1)
                return True
                
            else:
                print(f"Unknown gripper command: {command}")
                return False
                
        except Exception as e:
            print(f"Gripper control error: {e}")
            self.gripper_state = GripperState.ERROR
            return False
    
    def get_robot_state(self) -> RobotState:
        """Get current robot operational state"""
        return self.current_state
    
    def emergency_stop(self) -> bool:
        """Execute emergency stop"""
        self.current_state = RobotState.EMERGENCY_STOP
        self.gripper_state = GripperState.OPEN
        print("EMERGENCY STOP ACTIVATED")
        return True
    
    def get_capabilities(self) -> Dict:
        """Get robot capabilities and limits"""
        return self.capabilities.copy()
    
    def get_performance_metrics(self) -> Dict:
        """Get performance metrics"""
        total_ops = self.metrics['successful_operations'] + self.metrics['failed_operations']
        success_rate = (self.metrics['successful_operations'] / total_ops 
                       if total_ops > 0 else 0.0)
        
        return {
            **self.metrics,
            'total_operations': total_ops,
            'success_rate': success_rate,
            'current_state': self.current_state.value,
            'gripper_state': self.gripper_state.value
        }
    
    def _validate_pose(self, pose: RobotPose) -> bool:
        """Validate if pose is within robot workspace and safety limits"""
        x, y, z = pose.position
        bounds = self.safety_limits['workspace_bounds']
        
        if not (bounds['x_min'] <= x <= bounds['x_max']):
            return False
        if not (bounds['y_min'] <= y <= bounds['y_max']):
            return False
        if not (bounds['z_min'] <= z <= bounds['z_max']):
            return False
        
        # Check if position is within robot reach
        distance_from_base = np.sqrt(x**2 + y**2 + z**2)
        if distance_from_base > self.capabilities['max_reach']:
            return False
        
        return True
    
    def _calculate_pose_distance(self, pose1: RobotPose, pose2: RobotPose) -> float:
        """Calculate Euclidean distance between two poses"""
        return np.sqrt(sum((p1 - p2)**2 for p1, p2 in 
                          zip(pose1.position, pose2.position)))
    
    def _update_average_cycle_time(self):
        """Update average cycle time metric"""
        total_ops = self.metrics['successful_operations']
        if total_ops > 0:
            self.metrics['average_cycle_time'] = (self.metrics['total_operation_time'] / 
                                                 total_ops)

class RobotFactory:
    """Factory class for creating robot interfaces"""
    
    @staticmethod
    def create_robot(robot_type: str, robot_name: str = "robot") -> RobotInterface:
        """
        Create a robot interface of the specified type
        
        Args:
            robot_type: Type of robot to create
            robot_name: Name identifier for the robot
            
        Returns:
            RobotInterface implementation
        """
        if robot_type.lower() == "niryo":
            return NiryoRobotInterface(robot_name)
        else:
            raise ValueError(f"Unknown robot type: {robot_type}")
    
    @staticmethod
    def get_available_robots() -> List[str]:
        """Get list of available robot types"""
        return ["niryo"]