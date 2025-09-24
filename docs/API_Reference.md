# API Reference - Dyer-Maker Digital Twin

This document provides comprehensive API documentation for the Dyer-Maker Digital Twin system.

## Table of Contents

1. [Vision Interface API](#vision-interface-api)
2. [Robot Interface API](#robot-interface-api)
3. [Conveyor Interface API](#conveyor-interface-api)
4. [ROS2 Topics and Services](#ros2-topics-and-services)
5. [Configuration API](#configuration-api)
6. [Python Package API](#python-package-api)

## Vision Interface API

### VisionInterface (Abstract Base Class)

The base interface for all vision processing algorithms.

```python
from dyer_maker_digital_twin.interfaces.vision_interface import VisionInterface

class CustomVisionProcessor(VisionInterface):
    def configure(self, config: Dict) -> bool:
        """Configure the vision algorithm"""
        pass
    
    def process_frame(self, frame: np.ndarray) -> List[PartDetection]:
        """Process a single frame and return detected parts"""
        pass
    
    def get_diagnostics(self) -> Dict:
        """Get diagnostic information"""
        pass
    
    def calibrate(self, calibration_data: Dict) -> bool:
        """Calibrate the vision system"""
        pass
```

### PartDetection Data Class

```python
@dataclass
class PartDetection:
    color: PartColor                    # Detected color (RED, GREEN, BLUE, UNKNOWN)
    position: Tuple[float, float]       # (x, y) in image coordinates
    confidence: float                   # Detection confidence (0.0-1.0)
    timestamp: float                    # Unix timestamp
    bounding_box: Tuple[int, int, int, int]  # (x, y, width, height)
    world_position: Optional[Tuple[float, float, float]] = None  # (x, y, z) in world coordinates
```

### HSVColorDetector

Default implementation using HSV color space filtering.

```python
from dyer_maker_digital_twin.interfaces.vision_interface import HSVColorDetector

detector = HSVColorDetector()

# Configuration
config = {
    'color_ranges': {
        'red': {
            'lower1': [0, 50, 50],
            'upper1': [10, 255, 255]
        }
    },
    'min_contour_area': 500,
    'confidence_threshold': 0.7
}
detector.configure(config)

# Process frame
detections = detector.process_frame(image)
```

### VisionFactory

Factory for creating vision processors.

```python
from dyer_maker_digital_twin.interfaces.vision_interface import VisionFactory

# Create processor
processor = VisionFactory.create_processor("hsv_color")

# Get available algorithms
algorithms = VisionFactory.get_available_algorithms()
```

## Robot Interface API

### RobotInterface (Abstract Base Class)

Base interface for robot control systems.

```python
from dyer_maker_digital_twin.interfaces.robot_interface import RobotInterface

class CustomRobotController(RobotInterface):
    def initialize(self, config: Dict) -> bool:
        """Initialize the robot interface"""
        pass
    
    def move_to_pose(self, target_pose: RobotPose, speed: float = 0.1) -> bool:
        """Move robot to target pose"""
        pass
    
    def execute_pick_place(self, command: PickPlaceCommand) -> bool:
        """Execute pick and place operation"""
        pass
```

### Data Classes

```python
@dataclass
class RobotPose:
    position: Tuple[float, float, float]        # (x, y, z) in meters
    orientation: Tuple[float, float, float, float]  # Quaternion (x, y, z, w)
    timestamp: float

@dataclass
class PickPlaceCommand:
    pick_pose: RobotPose
    place_pose: RobotPose
    approach_offset: float = 0.05       # meters above target
    retreat_offset: float = 0.05        # meters above target
    pick_speed: float = 0.1            # m/s
    place_speed: float = 0.1           # m/s
    gripper_force: float = 50.0        # Newtons
```

### NiryoRobotInterface

Implementation for Niryo robots.

```python
from dyer_maker_digital_twin.interfaces.robot_interface import NiryoRobotInterface

robot = NiryoRobotInterface("niryo_ned2")

# Initialize
config = {
    'safety_limits': {
        'workspace_bounds': {
            'x_min': -0.6, 'x_max': 0.6,
            'y_min': -0.6, 'y_max': 0.6,
            'z_min': -0.1, 'z_max': 0.6
        }
    }
}
robot.initialize(config)

# Move to pose
target = RobotPose(
    position=(0.3, 0.0, 0.2),
    orientation=(0.0, 0.0, 0.0, 1.0),
    timestamp=time.time()
)
robot.move_to_pose(target, speed=0.1)

# Pick and place
command = PickPlaceCommand(
    pick_pose=pick_target,
    place_pose=place_target,
    gripper_force=50.0
)
robot.execute_pick_place(command)
```

### Enumerations

```python
class RobotState(Enum):
    IDLE = "idle"
    MOVING = "moving"
    PICKING = "picking"
    PLACING = "placing"
    ERROR = "error"
    EMERGENCY_STOP = "emergency_stop"

class GripperState(Enum):
    OPEN = "open"
    CLOSED = "closed"
    GRIPPING = "gripping"
    ERROR = "error"
```

## Conveyor Interface API

### ConveyorInterface (Abstract Base Class)

Base interface for conveyor control.

```python
from dyer_maker_digital_twin.interfaces.conveyor_interface import ConveyorInterface

class CustomConveyorController(ConveyorInterface):
    def start(self, speed: float, direction: ConveyorDirection) -> bool:
        """Start the conveyor belt"""
        pass
    
    def stop(self) -> bool:
        """Stop the conveyor belt"""
        pass
    
    def add_part(self, part: PartOnBelt) -> bool:
        """Add a part to belt tracking"""
        pass
```

### StandardConveyor

Default conveyor implementation.

```python
from dyer_maker_digital_twin.interfaces.conveyor_interface import StandardConveyor

conveyor = StandardConveyor("conveyor_1")

# Initialize
config = {
    'max_speed': 1.0,
    'belt_length': 2.0,
    'acceleration': 0.5
}
conveyor.initialize(config)

# Control
conveyor.start(speed=0.2, direction=ConveyorDirection.FORWARD)
conveyor.set_speed(0.5)
conveyor.stop()

# Part tracking
part = PartOnBelt(
    part_id="part_001",
    position=0.5,
    velocity=0.2,
    color="red",
    size=(0.05, 0.05, 0.02),
    timestamp_detected=time.time(),
    confidence=0.9
)
conveyor.add_part(part)
```

### Data Classes and Enums

```python
@dataclass
class ConveyorStatus:
    conveyor_id: str
    state: ConveyorState
    direction: ConveyorDirection
    speed: float                        # Current speed (m/s)
    target_speed: float                 # Target speed (m/s)
    belt_position: float                # Position from home (meters)
    parts_on_belt: List[Dict]          # Detected parts
    sensor_data: Dict                  # Sensor readings
    timestamp: float
    uptime: float                      # Seconds since start
    total_distance: float              # Total distance traveled

class ConveyorState(Enum):
    STOPPED = "stopped"
    RUNNING = "running"
    STARTING = "starting"
    STOPPING = "stopping"
    ERROR = "error"
    MAINTENANCE = "maintenance"

class ConveyorDirection(Enum):
    FORWARD = "forward"
    REVERSE = "reverse"
    STOPPED = "stopped"
```

## ROS2 Topics and Services

### Topics

#### Vision System
```bash
# Published by vision_node
/vision/part_detected          # Part detection results
/vision/status                 # Vision system status
/vision/debug_image           # Debug visualization
/vision/diagnostics           # Diagnostic information

# Subscribed by vision_node
/camera/image_raw             # Camera feed
```

#### Robot System
```bash
# Published by pick_place_node
/robot/status                 # Robot operational status
/robot/performance_metrics    # Performance data

# Subscribed by pick_place_node
/vision/part_detected         # Vision detection results
/system/commands              # High-level commands

# Standard robot topics
/joint_states                 # Joint state information
/tf                          # Transform tree
/tf_static                   # Static transforms
```

#### Conveyor System
```bash
# Published by conveyor_node
/conveyor/status              # Conveyor status
/conveyor/parts_tracking      # Part position tracking
/conveyor/diagnostics         # System diagnostics

# Subscribed by conveyor_node  
/system/commands              # Conveyor control commands
```

#### System Coordination
```bash
# Published by system_orchestrator_node
/system/state                 # Overall system state
/system/performance           # System performance metrics
/system/alerts               # System alerts and warnings

# Multi-directional
/system/commands             # System-wide commands
/emergency_stop              # Emergency stop signal
```

### Services

#### Vision Services
```bash
# Provided by vision_node
/vision/configure             # Update vision configuration
/vision/calibrate             # Calibrate vision system
/vision/get_diagnostics       # Get diagnostic information
```

#### Robot Services
```bash
# Provided by pick_place_node
/robot/move_to_pose          # Move to specific pose
/robot/execute_pick_place    # Execute pick and place
/robot/emergency_stop        # Emergency stop robot
/robot/get_capabilities      # Query robot capabilities
```

#### Conveyor Services
```bash
# Provided by conveyor_node
/conveyor/start              # Start conveyor
/conveyor/stop               # Stop conveyor
/conveyor/set_speed          # Change speed
/conveyor/add_part           # Add part to tracking
/conveyor/remove_part        # Remove part from tracking
```

#### System Services
```bash
# Provided by system_orchestrator_node
/system/start_demo           # Start automated demo
/system/pause_operations     # Pause all operations
/system/resume_operations    # Resume operations
/system/get_system_status    # Get comprehensive status
/system/emergency_shutdown   # Emergency shutdown all
```

## Configuration API

### Loading Configuration

```python
from dyer_maker_digital_twin.utils.config_manager import ConfigManager

# Load configuration
config = ConfigManager.load_config("config/vision_config.yaml")

# Access nested values
camera_topic = config.get('vision.camera.topic', '/camera/image_raw')
threshold = config.get('vision.processing.confidence_threshold', 0.7)

# Update configuration
config.set('vision.processing.confidence_threshold', 0.8)
ConfigManager.save_config(config, "config/vision_config.yaml")
```

### Configuration Validation

```python
from dyer_maker_digital_twin.utils.config_manager import ConfigValidator

# Define schema
schema = {
    'vision': {
        'algorithm_type': str,
        'processing': {
            'confidence_threshold': {'type': float, 'min': 0.0, 'max': 1.0}
        }
    }
}

# Validate configuration
validator = ConfigValidator(schema)
is_valid, errors = validator.validate(config)
```

## Python Package API

### Utilities

#### Performance Monitor
```python
from dyer_maker_digital_twin.utils.performance_monitor import PerformanceMonitor

monitor = PerformanceMonitor()

# Time operations
with monitor.time_operation("pick_place_cycle"):
    # Perform operation
    pass

# Get metrics
metrics = monitor.get_metrics()
print(f"Average cycle time: {metrics['pick_place_cycle']['avg_time']:.2f}s")
```

#### Diagnostics
```python
from dyer_maker_digital_twin.utils.diagnostics import DiagnosticsCollector

diagnostics = DiagnosticsCollector()

# Add diagnostic data
diagnostics.add_metric("fps", 30.0)
diagnostics.add_status("vision_system", "OK", "Running normally")

# Generate report
report = diagnostics.generate_report()
```

#### Config Manager
```python
from dyer_maker_digital_twin.utils.config_manager import ConfigManager

# Load with environment variable substitution
config = ConfigManager.load_config_with_env("config/robot_config.yaml")

# Merge configurations
base_config = ConfigManager.load_config("config/base.yaml")
override_config = ConfigManager.load_config("config/override.yaml")
merged = ConfigManager.merge_configs(base_config, override_config)

# Watch for changes
def on_config_change(config):
    print("Configuration updated!")

ConfigManager.watch_config("config/vision_config.yaml", on_config_change)
```

### Testing Utilities

#### Mock Interfaces
```python
from dyer_maker_digital_twin.test.mocks import MockVisionProcessor, MockRobotInterface

# Create mock vision processor
mock_vision = MockVisionProcessor()
mock_vision.set_detection_sequence([
    PartDetection(PartColor.RED, (100, 100), 0.9, time.time(), (90, 90, 20, 20)),
    PartDetection(PartColor.GREEN, (200, 100), 0.85, time.time(), (190, 90, 20, 20))
])

# Create mock robot
mock_robot = MockRobotInterface()
mock_robot.set_success_rate(0.95)  # 95% success rate for operations
```

#### Test Fixtures
```python
from dyer_maker_digital_twin.test.fixtures import VisionTestFixture

# Load test images and expected results
fixture = VisionTestFixture()
test_cases = fixture.load_test_cases("vision_test_data/")

for test_case in test_cases:
    result = vision_processor.process_frame(test_case.image)
    assert len(result) == len(test_case.expected_detections)
```

## Error Handling

### Exception Classes

```python
from dyer_maker_digital_twin.exceptions import (
    VisionProcessingError,
    RobotMotionError,
    ConveyorControlError,
    SystemConfigError
)

try:
    robot.move_to_pose(target_pose)
except RobotMotionError as e:
    logger.error(f"Motion failed: {e.message}")
    # Handle error appropriately
```

### Error Recovery

```python
from dyer_maker_digital_twin.utils.error_recovery import ErrorRecoveryManager

recovery_manager = ErrorRecoveryManager()

# Register recovery strategies
recovery_manager.register_strategy(
    RobotMotionError,
    lambda error: robot.move_to_home_position()
)

# Execute with automatic recovery
result = recovery_manager.execute_with_recovery(
    lambda: robot.execute_pick_place(command),
    max_retries=3
)
```

## Version Information

- **API Version**: 1.0.0
- **Compatible ROS2 Version**: Jazzy
- **Python Version**: 3.10+
- **Last Updated**: September 2025

For the most up-to-date API documentation, refer to the inline docstrings in the source code or generate documentation using Sphinx:

```bash
cd docs/
make html
```