# Lukebot Robot Specifications

## Overview
Lukebot is a custom mecanum wheel robot created in Isaac Sim for testing autonomous navigation and computer vision algorithms in various scenes.

## Physical Specifications

### Chassis
- **Dimensions**: 420mm (L) x 330mm (W) x 100mm (H)
- **Material**: Aluminum construction (simulated)
- **Mass**: 8.5 kg (base chassis)
- **Color**: Gray

### Wheels
- **Type**: Mecanum wheels (omnidirectional)
- **Quantity**: 4 wheels
- **Diameter**: 100mm (50mm radius)
- **Width**: 55mm
- **Configuration**:
  - Front Left: +45° roller angle
  - Front Right: -45° roller angle
  - Rear Left: -45° roller angle
  - Rear Right: +45° roller angle
- **Drive**: Continuous rotation joints with velocity control
- **Damping**: 100.0
- **Stiffness**: 0.0

### Wheelbase & Track
- **Wheelbase** (front to rear): 300mm
- **Track Width** (left to right): 340mm

## Sensors

### OAK-D IOT 75 Depth Camera
- **Model**: Mimics Luxonis OAK-D IOT 75
- **Mount Location**: Front of chassis, elevated on mount
- **Resolution**: 1280 x 800 pixels
- **Frame Rate**: 30 FPS
- **Horizontal FOV**: 75 degrees
- **Depth Range**: 0.35m to 10m
- **Focal Length**: Calculated from FOV
- **Horizontal Aperture**: 20.955mm
- **Physical Dimensions**: 34mm x 97mm x 28mm
- **Weight**: 84g

### IMU (Inertial Measurement Unit)
- **Location**: Center of chassis
- **Frequency**: 100 Hz
- **Outputs**: Linear acceleration (x, y, z), Angular velocity

## Capabilities

### Motion
- **Drive Type**: Holonomic (omnidirectional)
- **Movements**:
  - Forward/backward translation
  - Left/right strafing
  - Rotation in place
  - Combined diagonal movements
- **Control**: Velocity-based wheel control

### Perception
- **RGB-D Vision**: Full color images with depth data
- **Obstacle Detection**: Real-time depth sensing
- **Range**: 0.35m - 10m effective depth sensing

## File Locations

### URDF Model
```
source/extensions/isaacsim.asset.importer.urdf/data/urdf/robots/lukebot/urdf/lukebot.urdf
```

### Demo Scripts
```
source/standalone_examples/api/isaacsim.robot.wheeled_robots/lukebot_demo.py
source/standalone_examples/api/isaacsim.robot.wheeled_robots/lukebot_simple.py (simplified version)
source/standalone_examples/api/isaacsim.robot.wheeled_robots/lukebot_holonomic.py (with HolonomicController)
source/standalone_examples/api/isaacsim.robot.wheeled_robots/lukebot_teleop.py (keyboard WASD control)
```

## Running the Simulation

### Basic Demo
```bash
_build/windows-x86_64/release/python.bat source/standalone_examples/api/isaacsim.robot.wheeled_robots/lukebot_demo.py
```

### HolonomicController Demo (NEW - Automated Movement)
Demonstrates omnidirectional movement with the HolonomicController:
```bash
_build/windows-x86_64/release/python.bat source/standalone_examples/api/isaacsim.robot.wheeled_robots/lukebot_holonomic.py
```

### Keyboard Teleop Demo (NEW - Manual Control)
Control Lukebot with WASD keys:
- W/S: Forward/Backward
- A/D: Strafe Left/Right
- Q/E: Rotate CCW/CW
- SPACE: Stop
```bash
_build/windows-x86_64/release/python.bat source/standalone_examples/api/isaacsim.robot.wheeled_robots/lukebot_teleop.py
```

## URDF Structure

### Links
- `chassis_link` - Main body
- `front_left_wheel` - FL mecanum wheel
- `front_right_wheel` - FR mecanum wheel
- `rear_left_wheel` - RL mecanum wheel
- `rear_right_wheel` - RR mecanum wheel
- `camera_mount` - Camera mounting bracket
- `oak_d_camera` - Depth camera
- `oak_d_camera_optical` - Camera optical frame (ROS convention)
- `imu_link` - IMU sensor

### Joints
- `front_left_wheel_joint` - Continuous joint
- `front_right_wheel_joint` - Continuous joint
- `rear_left_wheel_joint` - Continuous joint
- `rear_right_wheel_joint` - Continuous joint
- `camera_mount_joint` - Fixed joint
- `camera_joint` - Fixed joint
- `camera_optical_joint` - Fixed joint (ROS convention)
- `imu_joint` - Fixed joint

### Custom Attributes (Mecanum Wheels)
Each wheel joint has:
- `isaacmecanumwheel:radius` = 0.05 (meters)
- `isaacmecanumwheel:angle` = ±π/4 radians (±45°)

## Test Scene

The demo includes test obstacles:
- **Box 1**: Red box at (1.5, 0.8, 0.25), size 0.4x0.4x0.5m
- **Box 2**: Green box at (-1.2, -1.2, 0.2), size 0.3x0.3x0.4m
- **Cylinder**: Blue cylinder at (0.8, -1.5, 0.3), radius 0.15m, height 0.3m
- **Wall**: Gray wall at (0.0, 2.5, 0.4), size 3.0x0.1x0.8m
- **Ground Plane**: Standard Isaac Sim ground plane

## Use Cases

1. **Autonomous Navigation Testing**: Test path planning algorithms with omnidirectional motion
2. **Obstacle Avoidance**: Use depth camera for real-time obstacle detection
3. **SLAM (Simultaneous Localization and Mapping)**: IMU + camera data fusion
4. **Computer Vision**: RGB-D processing for object detection/recognition
5. **Motion Control**: Test holonomic drive controllers
6. **Sensor Fusion**: Combine IMU and camera data

## Recent Updates (2025-10-16)

### Completed Features
1. ✅ **HolonomicController Integration** - Full omnidirectional control using Isaac Sim's native controller
2. ✅ **Keyboard Teleoperation** - WASD controls for manual robot operation
3. ✅ **Automated Movement Demo** - Scripted forward, strafe, and rotation demonstrations

### Controller Details
**HolonomicController** (`isaacsim.robot.wheeled_robots.controllers.holonomic_controller`):
- Quadratic programming-based wheel velocity computation
- Input: [forward_speed, lateral_speed, rotation_speed]
- Automatically handles mecanum wheel kinematics
- Optimizes wheel commands to minimize residual forces

**Keyboard Controls**:
- W: Forward movement
- S: Backward movement
- A: Strafe left
- D: Strafe right
- Q: Rotate counter-clockwise
- E: Rotate clockwise
- SPACE: Emergency stop

## Next Steps / TODOs

1. ✅ ~~Add holonomic controller to the demo script for autonomous motion~~
2. ✅ ~~Add teleoperation interface (keyboard/gamepad control)~~
3. **Fix camera integration** - Convert URDF link to proper Camera prim for OAK-D
4. **Create multiple test scenes** with different obstacle configurations
5. **Implement autonomous navigation** - Use camera to find yellow target cube
6. **Add obstacle avoidance** algorithm using depth camera
7. **Add ROS2 bridge** for external control and data streaming
8. **Create navigation benchmarks** for performance testing
9. **Add data logging** for camera and IMU outputs
10. **Implement SLAM** using camera and IMU data

## Known Issues

1. **Camera integration pending** - URDF imports create visual links, not Camera prims. Need to add camera conversion step for OAK-D sensor to work
2. Some deprecation warnings for `omni.isaac.sensor` (use `isaacsim.sensors.*` instead)

## Reference Product

Based on: [Mecanum Wheel Robot Chassis](https://www.amazon.com/Mecanum-Wheel-Chassis-Aluminum-Education/dp/B09CPZ51N4)

## Isaac Sim Version

- **Version**: 5.1.0
- **Build**: windows-x86_64/release
- **Platform**: Windows 11

---

Created: 2025-10-16
Last Modified: 2025-10-16
