# Dyer-Maker Digital Twin - Resilient Manufacturing Testbed

A modular ROS2-based digital twin for resilient manufacturing research, featuring dynamic job shop scheduling (DJSS), multi-robot coordination, and fault-tolerant operations.

## 🚀 Quick Start

### Prerequisites
- Ubuntu 24.04 LTS
- ROS2 Jazzy
- Gazebo (latest version)
- Python 3.10+
- OpenCV 4.x

### Installation

1. **Clone and setup ROS2 workspace:**
```bash
cd ~/ros2_drivers_ws/src
git clone <repository-url> dyer_maker_digital_twin
cd ~/ros2_drivers_ws
```

2. **Install dependencies:**
```bash
rosdep install --from-paths src --ignore-src -r -y
```

3. **Build the package:**
```bash
colcon build --packages-select dyer_maker_digital_twin
source install/setup.bash
```

4. **Set environment variables:**
```bash
export GZ_SIM_RESOURCE_PATH=~/ros2_drivers_ws/install/niryo_ned_description/share:$GZ_SIM_RESOURCE_PATH
```

### Running the MVP

**Single Robot Pick-and-Place Demo:**
```bash
ros2 launch dyer_maker_digital_twin single_robot_demo.launch.py
```

**Multi-Robot Testbed:**
```bash
ros2 launch dyer_maker_digital_twin multi_robot_testbed.launch.py
```

## 📋 Project Overview

### MVP Functionality
The current MVP demonstrates a complete pick-and-place workflow:

1. **Vision Detection**: Camera detects colored parts (red/green/blue) on conveyor belt
2. **Robot Coordination**: Niryo Ned2 robot picks up detected parts
3. **Transfer Operation**: Robot places parts on destination conveyor
4. **System Monitoring**: Real-time status and performance monitoring

### Key Features
- ✅ Modular ROS2 architecture
- ✅ Computer vision-based part detection
- ✅ MoveIt2 motion planning integration
- ✅ Configurable conveyor systems
- ✅ Multi-robot support (template)
- ✅ Comprehensive error handling
- ✅ Performance monitoring and diagnostics

## 🏗️ System Architecture

### Core Components

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Vision Node   │    │ Sensor Coord.    │    │ Pick-Place Node │
│                 │────│     Node         │────│                 │
│ - Color detect  │    │ - Data fusion    │    │ - Motion plan   │
│ - Part tracking │    │ - Validation     │    │ - Gripper ctrl  │
└─────────────────┘    └──────────────────┘    └─────────────────┘
         │                       │                       │
         └───────────────────────┼───────────────────────┘
                                 │
                    ┌──────────────────┐
                    │ System Orchestr. │
                    │     Node         │
                    │ - State machine  │
                    │ - Safety monitor │
                    │ - Performance    │
                    └──────────────────┘
```

### ROS2 Topics
- `/vision/part_detected` - Part detection results
- `/robot/joint_states` - Robot state information
- `/conveyor/status` - Conveyor system status
- `/system/commands` - High-level system commands
- `/pick_place/commands` - Robot operation commands

## 📁 Project Structure

```
dyer_maker_digital_twin/
├── launch/                 # Launch files
│   ├── single_robot_demo.launch.py
│   ├── multi_robot_testbed.launch.py
│   └── development.launch.py
├── dyer_maker_digital_twin/           # Python package
│   ├── nodes/             # ROS2 node implementations
│   │   ├── vision_node.py
│   │   ├── pick_place_node.py
│   │   ├── sensor_coordinator_node.py
│   │   └── system_orchestrator_node.py
│   ├── interfaces/        # Modular interfaces
│   │   ├── vision_interface.py
│   │   ├── robot_interface.py
│   │   └── conveyor_interface.py
│   └── utils/            # Utility modules
│       ├── config_manager.py
│       ├── diagnostics.py
│       └── performance_monitor.py
├── worlds/               # Gazebo world files
│   ├── factory_testbed.world
│   └── development.world
├── models/               # 3D models and SDF files
│   ├── conveyors/
│   ├── robots/
│   ├── parts/
│   └── sensors/
├── config/               # Configuration files
│   ├── vision_config.yaml
│   ├── robot_config.yaml
│   └── system_config.yaml
├── test/                # Unit and integration tests
├── docs/                # Documentation
│   ├── API_Reference.md
│   ├── Configuration_Guide.md
│   └── Troubleshooting.md
├── templates/           # Template files for expansion
├── package.xml          # ROS2 package definition
├── setup.py            # Python package setup
├── README.md           # This file
└── .gitignore          # Git ignore rules
```

## 🔧 Configuration

### Vision System
Edit `config/vision_config.yaml` to adjust:
- Color detection thresholds (HSV ranges)
- Camera parameters and resolution
- Detection confidence thresholds
- Processing frame rate

### Robot System
Edit `config/robot_config.yaml` to configure:
- Motion planning parameters
- Gripper force settings
- Safety limits and constraints
- Home positions and waypoints

### Conveyor System
Edit `config/conveyor_config.yaml` for:
- Belt speed and direction
- Part detection zones
- Timing synchronization
- Multiple conveyor coordination

## 🧪 Testing

### Run Unit Tests
```bash
cd ~/ros2_drivers_ws
colcon test --packages-select dyer_maker_digital_twin
colcon test-result --verbose
```

### Integration Testing
```bash
# Start test environment
ros2 launch dyer_maker_digital_twin test_environment.launch.py

# Run specific test scenarios
ros2 run dyer_maker_digital_twin test_vision_detection.py
ros2 run dyer_maker_digital_twin test_pick_place.py
ros2 run dyer_maker_digital_twin test_system_integration.py
```

### Performance Benchmarking
```bash
ros2 run dyer_maker_digital_twin benchmark_performance.py
```

## 📊 Monitoring and Diagnostics

### Real-time Monitoring
```bash
# System health dashboard
ros2 run dyer_maker_digital_twin health_monitor.py

# Performance metrics
ros2 topic echo /system/performance_metrics

# Vision diagnostics
ros2 service call /vision/get_diagnostics
```

### Data Logging
```bash
# Start data logging
ros2 bag record -a -o testbed_data

# Analyze logged data
ros2 run dyer_maker_digital_twin analyze_performance.py testbed_data
```

## 🔧 Development and Extension

### Adding New Vision Algorithms
1. Create new class implementing `VisionInterface`
2. Add configuration parameters to `vision_config.yaml`
3. Update vision node to load new algorithm
4. Test with development launch file

### Expanding Robot Capabilities
1. Implement new behaviors in `robot_interface.py`
2. Update motion planning parameters
3. Add safety constraints and validation
4. Test with single robot demo

### Multi-Robot Coordination
1. Use template files in `templates/multi_robot/`
2. Configure robot namespaces and positions
3. Implement coordination algorithms
4. Test with multi-robot launch file

## 🔍 Troubleshooting

### Common Issues

**Vision Detection Not Working:**
- Check camera topic: `ros2 topic list | grep camera`
- Verify color thresholds in config file
- Test with: `ros2 run dyer_maker_digital_twin test_vision.py`

**Robot Not Moving:**
- Check joint states: `ros2 topic echo /joint_states`
- Verify MoveIt planning scene
- Test with: `ros2 run dyer_maker_digital_twin test_robot.py`

**Simulation Performance Issues:**
- Reduce physics update rate in world file
- Disable unnecessary sensors
- Check system resources: `htop`

### Debug Mode
```bash
# Start in debug mode with verbose logging
ros2 launch dyer_maker_digital_twin single_robot_demo.launch.py debug:=true

# View debug logs
ros2 log set_logger_level dyer_maker_digital_twin DEBUG
```

## 📚 Documentation

- [MVP Proposal](MVP_Proposal.md) - Comprehensive project proposal
- [API Reference](docs/API_Reference.md) - Detailed API documentation
- [Configuration Guide](docs/Configuration_Guide.md) - Configuration options
- [Development Guide](docs/Development_Guide.md) - Extension and customization
- [Troubleshooting Guide](docs/Troubleshooting.md) - Common issues and solutions

## 🤝 Contributing

### Development Workflow
1. Fork the repository
2. Create feature branch: `git checkout -b feature/new-capability`
3. Implement changes with tests
4. Update documentation
5. Submit pull request

### Code Standards
- Follow ROS2 Python style guide
- Add type hints and docstrings
- Include unit tests for new features
- Update configuration examples

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- Niryo Robotics for robot models and documentation
- ROS2 community for excellent tools and libraries
- Gazebo development team for simulation platform
- OpenCV community for computer vision capabilities

## 📞 Support

For questions and support:
- Create an issue on GitHub
- Check documentation in `docs/` directory
- Review troubleshooting guide
- Contact development team

---

**Last Updated:** September 2025  
**Version:** MVP 1.0  
**Compatibility:** ROS2 Jazzy, Ubuntu 24.04, Gazebo (latest)