# Dyer-Maker

A ROS2 robotics project featuring the Niryo NED2 robot.

## Getting Started

### Launch the Robot

To launch the robot simulation:

```bash
ros2 launch launch/robot.launch.py
```

### Control Commands

#### Move Joint 1

To move joint 1 of the robot:

```bash
gz topic -t /model/niryo_ned2/joint/joint_1/0/cmd_pos -m gz.msgs.Double -p 'data: 1.0'
```

This command publishes a position command to joint 1, setting its position to 1.0 radians.

## Project Structure

- `launch/` - Launch files for the robot
- `models/robots/ned2/` - NED2 robot model files including URDF, configuration, and logic