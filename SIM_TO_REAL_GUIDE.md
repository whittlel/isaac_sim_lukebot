# Lukebot Sim-to-Real Transfer Guide

## Overview

The new motion controller architecture is designed for easy transfer from Isaac Sim simulation to your real Jetson Orin robot.

## Architecture

```
┌─────────────────────────────────────────────────┐
│         High-Level Application Code             │
│  (Navigation, Computer Vision, Path Planning)   │
└────────────────┬────────────────────────────────┘
                 │
                 │ motion_controller.set_velocity(vx, vy, omega)
                 │
┌────────────────▼────────────────────────────────┐
│         MotionController Interface              │
│     (Abstract - same for sim and real)          │
└────────────────┬────────────────────────────────┘
                 │
      ┌──────────┴──────────┐
      │                     │
┌─────▼─────────┐  ┌───────▼────────────┐
│ Simulation    │  │  Real Robot        │
│ Controller    │  │  Controller        │
│ (Isaac Sim)   │  │  (Jetson Orin)     │
└───────────────┘  └────────────────────┘
```

## Key Files

### 1. `lukebot_motion_controller.py`
The motion controller abstraction with three main classes:

- **`MotionController`** (Abstract Base Class)
  - Interface that works everywhere
  - Methods: `set_velocity(vx, vy, omega)`, `reset()`, `get_actual_velocity()`

- **`MecanumKinematics`**
  - Pure math - works in both sim and real
  - Converts robot velocity ↔ wheel velocities
  - No dependencies on Isaac Sim or ROS

- **`SimulationMotionController`**
  - Uses Isaac Sim's Robot class
  - For testing in simulation

- **`RealRobotMotionController`**
  - STUB - implement for Jetson
  - Where you'll add ROS 2, motor controllers, etc.

### 2. `lukebot_teleop_v2.py`
Example usage - shows how simple the interface is:

```python
# Create controller (same interface for sim and real!)
motion_controller = create_motion_controller(
    robot=my_lukebot,      # Only needed in simulation
    use_simulation=True,   # False on Jetson
    wheel_base=0.30,
    track_width=0.34,
    wheel_radius=0.05
)

# Control robot (SAME CODE works in sim and real!)
motion_controller.set_velocity(vx=0.5, vy=0.0, omega=0.0)  # Forward
motion_controller.set_velocity(vx=0.0, vy=0.5, omega=0.0)  # Strafe left
motion_controller.set_velocity(vx=0.0, vy=0.0, omega=1.0)  # Rotate
```

## How to Transfer to Real Robot

### Step 1: Copy the Motion Controller
Copy `lukebot_motion_controller.py` to your Jetson Orin.

### Step 2: Implement RealRobotMotionController

Edit the `RealRobotMotionController` class in `lukebot_motion_controller.py`:

```python
class RealRobotMotionController(MotionController):
    def __init__(self, kinematics: MecanumKinematics):
        self.kinematics = kinematics

        # Initialize your hardware
        import rclpy
        from geometry_msgs.msg import Twist

        rclpy.init()
        self.node = rclpy.create_node('lukebot_controller')
        self.cmd_vel_pub = self.node.create_publisher(Twist, 'cmd_vel', 10)

        # OR initialize motor controllers directly
        # from your_motor_library import MotorController
        # self.motors = MotorController()

    def set_velocity(self, vx: float, vy: float, omega: float) -> None:
        # Calculate wheel velocities (same math as simulation!)
        wheel_velocities = self.kinematics.inverse_kinematics(vx, vy, omega)

        # OPTION 1: Publish to ROS 2
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = omega
        self.cmd_vel_pub.publish(twist)

        # OPTION 2: Command motors directly
        # self.motors.set_wheel_speeds(wheel_velocities)

        # OPTION 3: Send CAN bus commands
        # self.can_bus.send_wheel_commands(wheel_velocities)
```

### Step 3: Update Your Application Code

Change one line:

```python
# In simulation
motion_controller = create_motion_controller(
    robot=my_robot,
    use_simulation=True,  # ← Simulation mode
    ...
)

# On Jetson (real robot)
motion_controller = create_motion_controller(
    use_simulation=False,  # ← Real robot mode
    ...
)

# Everything else stays the same!
motion_controller.set_velocity(vx, vy, omega)
```

## Benefits

### ✅ Sim-to-Real Transfer
- Train/test in Isaac Sim
- Deploy to Jetson with minimal changes
- Same high-level code

### ✅ Separation of Concerns
- **Kinematics**: Pure math, works everywhere
- **Application logic**: Doesn't know about hardware
- **Hardware interface**: Isolated to one class

### ✅ Testing
- Test navigation algorithms in sim
- Test vision algorithms in sim
- Only test motor control on real robot

### ✅ Flexibility
- Swap between sim and real easily
- Test different control strategies
- Easy to add new features

## Mecanum Wheel Kinematics

The math used is the standard mecanum inverse kinematics for X-configuration:

```
Wheel Layout (top view):
     FRONT
       ↑
   [FL][FR]    FL: Front Left wheel
   [RL][RR]    FR: Front Right wheel
               RL: Rear Left wheel
               RR: Rear Right wheel

Roller angles:
- FL: -45° (rollers lean forward-right)
- FR: +45° (rollers lean forward-left)
- RL: +45° (rollers lean backward-right)
- RR: -45° (rollers lean backward-left)

Inverse Kinematics (robot velocity → wheel velocities):
  ω_FL = (vx - vy - (lx+ly)*ω) / r
  ω_FR = (vx + vy + (lx+ly)*ω) / r
  ω_RL = (vx + vy - (lx+ly)*ω) / r
  ω_RR = (vx - vy + (lx+ly)*ω) / r

Where:
  vx = forward velocity (m/s)
  vy = lateral velocity (m/s)
  ω = angular velocity (rad/s)
  lx = wheel_base / 2
  ly = track_width / 2
  r = wheel_radius
```

## Example: Adding Camera-Based Navigation

```python
# Works the same in sim and real!

from lukebot_motion_controller import create_motion_controller
import cv2

# Create controller
controller = create_motion_controller(use_simulation=USE_SIM)

# Process camera image
image = get_camera_image()
target_x, target_y = detect_yellow_cube(image)

# Calculate motion command
vx, vy, omega = calculate_approach_velocity(target_x, target_y)

# Command robot (same code for sim and real!)
controller.set_velocity(vx, vy, omega)
```

## Robot Specifications

Current configuration in code:
- **wheel_base**: 0.30 m (300mm between front/rear axles)
- **track_width**: 0.34 m (340mm between left/right wheels)
- **wheel_radius**: 0.05 m (50mm diameter wheels = 100mm)

Verify these match your physical robot and update if needed.

## Testing Checklist

### In Simulation
- [x] Forward/backward movement
- [x] Left/right strafing
- [x] Rotation in place
- [x] Diagonal movement
- [x] Combined movements

### On Real Robot
- [ ] Calibrate wheel dimensions
- [ ] Test forward/backward
- [ ] Test strafing
- [ ] Test rotation
- [ ] Tune motor PID controllers
- [ ] Test on different surfaces

## Troubleshooting

### Simulation Issues
- Make sure you're using `lukebot_teleop_v2.py`, not the old version
- Check that `lukebot_motion_controller.py` is in the same directory

### Real Robot Issues
- Verify wheel dimensions match physical robot
- Check motor controller wiring
- Ensure wheel velocities have correct signs
- Calibrate IMU if available

## Next Steps

1. **Test in simulation** with `run_teleop_v2.bat`
2. **Verify all movements work**
3. **Copy to Jetson** when ready
4. **Implement `RealRobotMotionController`**
5. **Test on real hardware**

The beauty of this architecture: your navigation, vision, and AI code doesn't change!
