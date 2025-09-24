#!/usr/bin/env python3
"""
Conveyor Interface Module for Dyer-Maker Digital Twin

This module provides a modular interface for conveyor belt control and monitoring,
allowing integration with different conveyor systems and control hardware.

Author: Dyer-Maker Team
License: MIT
"""

from abc import ABC, abstractmethod
from typing import Dict, List, Tuple, Optional
import numpy as np
from dataclasses import dataclass
from enum import Enum
import time
import threading

class ConveyorState(Enum):
    """Conveyor operational states"""
    STOPPED = "stopped"
    RUNNING = "running"
    STARTING = "starting"
    STOPPING = "stopping"
    ERROR = "error"
    MAINTENANCE = "maintenance"

class ConveyorDirection(Enum):
    """Conveyor movement direction"""
    FORWARD = "forward"
    REVERSE = "reverse"
    STOPPED = "stopped"

@dataclass
class ConveyorStatus:
    """Conveyor system status"""
    conveyor_id: str
    state: ConveyorState
    direction: ConveyorDirection
    speed: float  # m/s
    target_speed: float  # m/s
    belt_position: float  # meters from home
    parts_on_belt: List[Dict]  # List of detected parts
    sensor_data: Dict
    timestamp: float
    uptime: float  # seconds since last start
    total_distance: float  # total belt travel distance

@dataclass
class PartOnBelt:
    """Information about a part on the conveyor belt"""
    part_id: str
    position: float  # position along belt (meters)
    velocity: float  # m/s relative to belt
    color: str
    size: Tuple[float, float, float]  # (length, width, height)
    timestamp_detected: float
    confidence: float

class ConveyorInterface(ABC):
    """Abstract base class for conveyor control interfaces"""
    
    @abstractmethod
    def initialize(self, config: Dict) -> bool:
        """
        Initialize the conveyor interface
        
        Args:
            config: Configuration parameters
            
        Returns:
            bool: True if initialization successful
        """
        pass
    
    @abstractmethod
    def start(self, speed: float, direction: ConveyorDirection = ConveyorDirection.FORWARD) -> bool:
        """
        Start the conveyor belt
        
        Args:
            speed: Belt speed in m/s
            direction: Movement direction
            
        Returns:
            bool: True if start successful
        """
        pass
    
    @abstractmethod
    def stop(self) -> bool:
        """
        Stop the conveyor belt
        
        Returns:
            bool: True if stop successful
        """
        pass
    
    @abstractmethod
    def set_speed(self, speed: float) -> bool:
        """
        Change conveyor speed
        
        Args:
            speed: New speed in m/s
            
        Returns:
            bool: True if speed change successful
        """
        pass
    
    @abstractmethod
    def get_status(self) -> ConveyorStatus:
        """Get current conveyor status"""
        pass
    
    @abstractmethod
    def add_part(self, part: PartOnBelt) -> bool:
        """
        Add a part to the belt tracking system
        
        Args:
            part: Part information
            
        Returns:
            bool: True if part added successfully
        """
        pass
    
    @abstractmethod
    def remove_part(self, part_id: str) -> bool:
        """
        Remove a part from belt tracking
        
        Args:
            part_id: Unique part identifier
            
        Returns:
            bool: True if part removed successfully
        """
        pass
    
    @abstractmethod
    def get_parts_in_zone(self, start_pos: float, end_pos: float) -> List[PartOnBelt]:
        """
        Get parts within a specific zone on the belt
        
        Args:
            start_pos: Start position (meters)
            end_pos: End position (meters)
            
        Returns:
            List of parts in the specified zone
        """
        pass
    
    @abstractmethod
    def emergency_stop(self) -> bool:
        """Execute emergency stop"""
        pass

class StandardConveyor(ConveyorInterface):
    """
    Standard conveyor implementation
    
    This implementation provides control for a typical industrial conveyor
    belt with variable speed control and part tracking capabilities.
    """
    
    def __init__(self, conveyor_id: str = "conveyor_1"):
        """Initialize standard conveyor"""
        self.conveyor_id = conveyor_id
        self.state = ConveyorState.STOPPED
        self.direction = ConveyorDirection.STOPPED
        
        # Physical parameters
        self.current_speed = 0.0  # m/s
        self.target_speed = 0.0   # m/s
        self.max_speed = 1.0      # m/s
        self.acceleration = 0.5   # m/s²
        self.deceleration = 0.5   # m/s²
        self.belt_length = 2.0    # meters
        self.belt_width = 0.3     # meters
        
        # Position tracking
        self.belt_position = 0.0  # current position
        self.total_distance = 0.0 # cumulative distance
        self.start_time = 0.0
        self.last_update_time = time.time()
        
        # Part tracking
        self.parts_on_belt = {}  # dict of part_id -> PartOnBelt
        self.part_counter = 0
        
        # Sensor simulation
        self.sensor_data = {
            'photo_eye_entry': False,
            'photo_eye_exit': False,
            'encoder_position': 0.0,
            'motor_current': 0.0,
            'belt_tension': 100.0,
            'temperature': 25.0
        }
        
        # Control thread
        self._running = False
        self._control_thread = None
        self._control_lock = threading.Lock()
        
        # Performance metrics
        self.metrics = {
            'uptime': 0.0,
            'total_runtime': 0.0,
            'parts_processed': 0,
            'emergency_stops': 0,
            'fault_count': 0
        }
    
    def initialize(self, config: Dict) -> bool:
        """Initialize the conveyor system"""
        try:
            # Update configuration parameters
            if 'conveyor_id' in config:
                self.conveyor_id = config['conveyor_id']
            
            if 'max_speed' in config:
                self.max_speed = config['max_speed']
            
            if 'acceleration' in config:
                self.acceleration = config['acceleration']
                
            if 'deceleration' in config:
                self.deceleration = config['deceleration']
                
            if 'belt_length' in config:
                self.belt_length = config['belt_length']
                
            if 'belt_width' in config:
                self.belt_width = config['belt_width']
            
            # Initialize control systems
            self._initialize_motor_control()
            self._initialize_sensors()
            
            # Start control loop
            self._start_control_loop()
            
            print(f"Conveyor {self.conveyor_id} initialized successfully")
            return True
            
        except Exception as e:
            print(f"Conveyor initialization error: {e}")
            self.state = ConveyorState.ERROR
            return False
    
    def _initialize_motor_control(self):
        """Initialize motor control system"""
        # Placeholder for motor control initialization
        # In real implementation, this would initialize:
        # - Motor driver interface
        # - Speed control PID
        # - Encoder feedback
        print(f"Initializing motor control for {self.conveyor_id}")
    
    def _initialize_sensors(self):
        """Initialize sensor systems"""
        # Placeholder for sensor initialization
        # In real implementation, this would initialize:
        # - Photo eyes and proximity sensors
        # - Encoders and position feedback
        # - Temperature and diagnostic sensors
        print(f"Initializing sensors for {self.conveyor_id}")
    
    def _start_control_loop(self):
        """Start the conveyor control loop thread"""
        if not self._running:
            self._running = True
            self._control_thread = threading.Thread(target=self._control_loop, daemon=True)
            self._control_thread.start()
    
    def _control_loop(self):
        """Main control loop for conveyor operation"""
        while self._running:
            try:
                current_time = time.time()
                dt = current_time - self.last_update_time
                
                with self._control_lock:
                    # Update speed control
                    self._update_speed_control(dt)
                    
                    # Update position tracking
                    self._update_position_tracking(dt)
                    
                    # Update part tracking
                    self._update_part_tracking(dt)
                    
                    # Update sensors
                    self._update_sensors()
                    
                    # Update metrics
                    self._update_metrics(dt)
                
                self.last_update_time = current_time
                time.sleep(0.01)  # 100Hz control loop
                
            except Exception as e:
                print(f"Control loop error: {e}")
                self.state = ConveyorState.ERROR
    
    def _update_speed_control(self, dt: float):
        """Update speed control with acceleration/deceleration limits"""
        if self.state in [ConveyorState.STARTING, ConveyorState.RUNNING]:
            # Accelerate towards target speed
            speed_error = self.target_speed - self.current_speed
            
            if abs(speed_error) > 0.001:  # Small threshold to avoid oscillation
                if speed_error > 0:
                    # Accelerating
                    speed_change = min(self.acceleration * dt, speed_error)
                else:
                    # Decelerating
                    speed_change = max(-self.deceleration * dt, speed_error)
                
                self.current_speed += speed_change
                
                # Update state based on speed achievement
                if abs(speed_error) < 0.001:
                    if self.target_speed > 0:
                        self.state = ConveyorState.RUNNING
                    else:
                        self.state = ConveyorState.STOPPED
                        self.direction = ConveyorDirection.STOPPED
                        
        elif self.state == ConveyorState.STOPPING:
            # Decelerate to stop
            if self.current_speed > 0.001:
                speed_change = min(self.deceleration * dt, self.current_speed)
                self.current_speed -= speed_change
            else:
                self.current_speed = 0.0
                self.state = ConveyorState.STOPPED
                self.direction = ConveyorDirection.STOPPED
    
    def _update_position_tracking(self, dt: float):
        """Update belt position and distance tracking"""
        if self.current_speed > 0:
            distance_moved = self.current_speed * dt
            
            if self.direction == ConveyorDirection.FORWARD:
                self.belt_position += distance_moved
            elif self.direction == ConveyorDirection.REVERSE:
                self.belt_position -= distance_moved
            
            # Wrap position for continuous belt
            self.belt_position = self.belt_position % self.belt_length
            self.total_distance += abs(distance_moved)
    
    def _update_part_tracking(self, dt: float):
        """Update positions of parts on the belt"""
        parts_to_remove = []
        
        for part_id, part in self.parts_on_belt.items():
            if self.current_speed > 0:
                # Update part position based on belt movement
                if self.direction == ConveyorDirection.FORWARD:
                    part.position += self.current_speed * dt
                elif self.direction == ConveyorDirection.REVERSE:
                    part.position -= self.current_speed * dt
                
                # Remove parts that have moved off the belt
                if part.position < 0 or part.position > self.belt_length:
                    parts_to_remove.append(part_id)
        
        # Remove parts that are off the belt
        for part_id in parts_to_remove:
            del self.parts_on_belt[part_id]
            self.metrics['parts_processed'] += 1
    
    def _update_sensors(self):
        """Update simulated sensor readings"""
        # Photo eye sensors
        self.sensor_data['photo_eye_entry'] = any(
            0.1 <= part.position <= 0.2 for part in self.parts_on_belt.values()
        )
        
        self.sensor_data['photo_eye_exit'] = any(
            self.belt_length - 0.2 <= part.position <= self.belt_length - 0.1 
            for part in self.parts_on_belt.values()
        )
        
        # Encoder position
        self.sensor_data['encoder_position'] = self.belt_position
        
        # Motor current (proportional to speed)
        self.sensor_data['motor_current'] = self.current_speed * 2.0  # Simulated
        
        # Belt tension and temperature
        self.sensor_data['belt_tension'] = 100.0 + np.random.normal(0, 2)  # Simulated
        self.sensor_data['temperature'] = 25.0 + np.random.normal(0, 1)    # Simulated
    
    def _update_metrics(self, dt: float):
        """Update performance metrics"""
        if self.state in [ConveyorState.RUNNING, ConveyorState.STARTING, ConveyorState.STOPPING]:
            self.metrics['uptime'] += dt
        
        self.metrics['total_runtime'] += dt
    
    def start(self, speed: float, direction: ConveyorDirection = ConveyorDirection.FORWARD) -> bool:
        """Start the conveyor belt"""
        try:
            if self.state == ConveyorState.ERROR:
                print("Cannot start conveyor in error state")
                return False
            
            # Validate speed
            if speed < 0 or speed > self.max_speed:
                print(f"Speed {speed} outside valid range [0, {self.max_speed}]")
                return False
            
            with self._control_lock:
                self.target_speed = speed
                self.direction = direction
                self.state = ConveyorState.STARTING
                self.start_time = time.time()
            
            print(f"Starting conveyor {self.conveyor_id} at {speed} m/s {direction.value}")
            return True
            
        except Exception as e:
            print(f"Conveyor start error: {e}")
            self.state = ConveyorState.ERROR
            return False
    
    def stop(self) -> bool:
        """Stop the conveyor belt"""
        try:
            with self._control_lock:
                self.target_speed = 0.0
                self.state = ConveyorState.STOPPING
            
            print(f"Stopping conveyor {self.conveyor_id}")
            return True
            
        except Exception as e:
            print(f"Conveyor stop error: {e}")
            self.state = ConveyorState.ERROR
            return False
    
    def set_speed(self, speed: float) -> bool:
        """Change conveyor speed"""
        try:
            if speed < 0 or speed > self.max_speed:
                print(f"Speed {speed} outside valid range [0, {self.max_speed}]")
                return False
            
            with self._control_lock:
                self.target_speed = speed
                if speed > 0 and self.state == ConveyorState.STOPPED:
                    self.state = ConveyorState.STARTING
                elif speed == 0:
                    self.state = ConveyorState.STOPPING
            
            return True
            
        except Exception as e:
            print(f"Speed change error: {e}")
            return False
    
    def get_status(self) -> ConveyorStatus:
        """Get current conveyor status"""
        with self._control_lock:
            uptime = time.time() - self.start_time if self.start_time > 0 else 0
            
            parts_list = [
                {
                    'id': part.part_id,
                    'position': part.position,
                    'color': part.color,
                    'confidence': part.confidence
                }
                for part in self.parts_on_belt.values()
            ]
            
            return ConveyorStatus(
                conveyor_id=self.conveyor_id,
                state=self.state,
                direction=self.direction,
                speed=self.current_speed,
                target_speed=self.target_speed,
                belt_position=self.belt_position,
                parts_on_belt=parts_list,
                sensor_data=self.sensor_data.copy(),
                timestamp=time.time(),
                uptime=uptime,
                total_distance=self.total_distance
            )
    
    def add_part(self, part: PartOnBelt) -> bool:
        """Add a part to the belt tracking system"""
        try:
            with self._control_lock:
                self.parts_on_belt[part.part_id] = part
            return True
        except Exception as e:
            print(f"Error adding part: {e}")
            return False
    
    def remove_part(self, part_id: str) -> bool:
        """Remove a part from belt tracking"""
        try:
            with self._control_lock:
                if part_id in self.parts_on_belt:
                    del self.parts_on_belt[part_id]
                    return True
                else:
                    print(f"Part {part_id} not found on belt")
                    return False
        except Exception as e:
            print(f"Error removing part: {e}")
            return False
    
    def get_parts_in_zone(self, start_pos: float, end_pos: float) -> List[PartOnBelt]:
        """Get parts within a specific zone on the belt"""
        with self._control_lock:
            parts_in_zone = []
            for part in self.parts_on_belt.values():
                if start_pos <= part.position <= end_pos:
                    parts_in_zone.append(part)
            return parts_in_zone
    
    def emergency_stop(self) -> bool:
        """Execute emergency stop"""
        try:
            with self._control_lock:
                self.current_speed = 0.0
                self.target_speed = 0.0
                self.state = ConveyorState.STOPPED
                self.direction = ConveyorDirection.STOPPED
                self.metrics['emergency_stops'] += 1
            
            print(f"EMERGENCY STOP - Conveyor {self.conveyor_id}")
            return True
            
        except Exception as e:
            print(f"Emergency stop error: {e}")
            return False
    
    def get_performance_metrics(self) -> Dict:
        """Get performance metrics"""
        return {
            **self.metrics,
            'current_state': self.state.value,
            'current_speed': self.current_speed,
            'uptime_percentage': (self.metrics['uptime'] / self.metrics['total_runtime'] 
                                if self.metrics['total_runtime'] > 0 else 0.0),
            'parts_on_belt': len(self.parts_on_belt)
        }
    
    def shutdown(self):
        """Shutdown the conveyor system"""
        self._running = False
        if self._control_thread:
            self._control_thread.join(timeout=1.0)
        print(f"Conveyor {self.conveyor_id} shutdown complete")

class ConveyorFactory:
    """Factory class for creating conveyor interfaces"""
    
    @staticmethod
    def create_conveyor(conveyor_type: str, conveyor_id: str = "conveyor") -> ConveyorInterface:
        """
        Create a conveyor interface of the specified type
        
        Args:
            conveyor_type: Type of conveyor to create
            conveyor_id: Unique identifier for the conveyor
            
        Returns:
            ConveyorInterface implementation
        """
        if conveyor_type.lower() == "standard":
            return StandardConveyor(conveyor_id)
        else:
            raise ValueError(f"Unknown conveyor type: {conveyor_type}")
    
    @staticmethod
    def get_available_conveyors() -> List[str]:
        """Get list of available conveyor types"""
        return ["standard"]