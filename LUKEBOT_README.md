# Isaac Sim Lukebot - Mecanum Wheel Robot

A custom mecanum wheel robot implementation in NVIDIA Isaac Sim 5.1.0 featuring omnidirectional movement, OAK-D depth camera integration, and keyboard teleoperation.

![Isaac Sim Version](https://img.shields.io/badge/Isaac%20Sim-5.1.0-green)
![Platform](https://img.shields.io/badge/Platform-Windows%2011-blue)
![Python](https://img.shields.io/badge/Python-3.10-blue)

## Overview

Lukebot is a holonomic robot platform designed for testing autonomous navigation, computer vision, and mecanum wheel control algorithms in Isaac Sim. It features:

- **4x Mecanum Wheels** - Full omnidirectional movement capability
- **OAK-D IOT 75 Camera** - Depth perception and RGB imaging (75° FOV, 1280x800, 0.35-10m range)
- **IMU Sensor** - Inertial measurement for motion tracking
- **HolonomicController** - Native Isaac Sim controller for smooth mecanum drive
- **Keyboard Teleoperation** - WASD controls for manual operation

## Robot Specifications

### Physical Dimensions
- **Chassis**: 420mm (L) × 330mm (W) × 100mm (H)
- **Weight**: 8.5 kg base chassis + 1.4 kg wheels
- **Wheels**: 100mm diameter × 55mm width mecanum wheels
- **Wheelbase**: 300mm (front to rear)
- **Track Width**: 340mm (left to right)

### Capabilities
- **Motion**: Forward/backward, strafe left/right, rotate in place, diagonal movement
- **Vision**: RGB-D depth sensing, 75° horizontal FOV
- **Control**: Velocity-based holonomic control with optimized wheel coordination

## Quick Start

### Running the Demos

#### 1. Holonomic Controller Demo (Automated Movement)
Demonstrates programmed omnidirectional movement patterns:
```bash
_build/windows-x86_64/release/python.bat source/standalone_examples/api/isaacsim.robot.wheeled_robots/lukebot_holonomic.py
```

**Movement Pattern:**
- Frames 0-500: Forward (X+)
- Frames 500-1000: Strafe left (Y+)
- Frames 1000-1500: Strafe right (Y-)
- Frames 1500-1700: Rotate counter-clockwise

#### 2. Keyboard Teleoperation Demo
Control the robot manually with your keyboard:
```bash
_build/windows-x86_64/release/python.bat source/standalone_examples/api/isaacsim.robot.wheeled_robots/lukebot_teleop.py
```

**Keyboard Controls:**
| Key | Action |
|-----|--------|
| W | Move forward |
| S | Move backward |
| A | Strafe left |
| D | Strafe right |
| Q | Rotate counter-clockwise |
| E | Rotate clockwise |
| SPACE | Emergency stop |

#### 3. Basic URDF Import Demo
Simple demonstration of URDF import and scene setup:
```bash
_build/windows-x86_64/release/python.bat source/standalone_examples/api/isaacsim.robot.wheeled_robots/lukebot_demo.py
```

## Project Structure

```
.
├── LUKEBOT_README.md                                    # This file
├── LUKEBOT_SPECS.md                                     # Detailed robot specifications
│
├── source/extensions/isaacsim.asset.importer.urdf/data/urdf/robots/lukebot/
│   └── urdf/
│       └── lukebot.urdf                                 # Robot URDF definition
│
└── source/standalone_examples/api/isaacsim.robot.wheeled_robots/
    ├── lukebot_demo.py                                  # Basic demo
    ├── lukebot_simple.py                                # Simplified version
    ├── lukebot_holonomic.py                             # HolonomicController demo
    └── lukebot_teleop.py                                # Keyboard teleoperation
```

## Technical Details

### HolonomicController

Lukebot uses Isaac Sim's native `HolonomicController` which:
- Employs quadratic programming to compute optimal wheel velocities
- Takes input as `[forward_speed, lateral_speed, rotation_speed]`
- Automatically handles mecanum wheel kinematics
- Minimizes residual forces on the robot's center of mass

```python
from isaacsim.robot.wheeled_robots.controllers.holonomic_controller import HolonomicController

# Initialize controller
my_controller = HolonomicController(
    name="holonomic_controller",
    wheel_radius=wheel_radius,
    wheel_positions=wheel_positions,
    wheel_orientations=wheel_orientations,
    mecanum_angles=mecanum_angles,
    wheel_axis=wheel_axis,
    up_axis=up_axis,
)

# Apply movement command
my_robot.apply_wheel_actions(
    my_controller.forward(command=[0.3, 0.2, 0.1])  # [forward, lateral, rotation]
)
```

### Mecanum Wheel Configuration

Each wheel has custom Isaac Sim attributes:
```python
isaacmecanumwheel:radius = 0.05  # meters
isaacmecanumwheel:angle = ±π/4   # ±45° roller angle
```

**Wheel Angles:**
- Front Left: +45°
- Front Right: -45°
- Rear Left: -45°
- Rear Right: +45°

## Use Cases

1. **Autonomous Navigation** - Test path planning with omnidirectional motion
2. **Obstacle Avoidance** - Use depth camera for real-time obstacle detection
3. **SLAM** - Simultaneous localization and mapping with IMU + camera fusion
4. **Computer Vision** - RGB-D processing for object detection/recognition
5. **Motion Control** - Test and validate holonomic drive controllers
6. **Sensor Fusion** - Combine IMU and camera data for robust navigation

## Roadmap

### Completed ✅
- [x] URDF robot model
- [x] Mecanum wheel configuration
- [x] HolonomicController integration
- [x] Keyboard teleoperation
- [x] Test scene with obstacles

### In Progress 🚧
- [ ] OAK-D camera integration (URDF link → Camera prim conversion)
- [ ] Autonomous cube-finding navigation

### Planned 📋
- [ ] ROS2 bridge for external control
- [ ] Obstacle avoidance algorithm using depth data
- [ ] SLAM implementation
- [ ] Multiple test scenes
- [ ] Data logging for sensors
- [ ] Performance benchmarks

## Known Issues

1. **Camera Integration** - URDF imports create visual links instead of Camera prims. Camera sensor initialization is pending proper prim conversion.
2. **Deprecation Warnings** - Some `omni.isaac.sensor` warnings (use `isaacsim.sensors.*` modules instead).

## Hardware Reference

Based on: [Mecanum Wheel Robot Chassis](https://www.amazon.com/Mecanum-Wheel-Chassis-Aluminum-Education/dp/B09CPZ51N4)

---

**Created:** 2025-10-16
**Last Updated:** 2025-10-16
**Isaac Sim Version:** 5.1.0
**Author:** Luke (@whittlel)
