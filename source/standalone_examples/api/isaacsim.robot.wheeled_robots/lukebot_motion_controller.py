# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Lukebot Motion Controller - Abstraction layer for robot motion

This module provides an abstraction between high-level motion commands
and the actual robot control. This allows for easy swapping between:
- Simulation control (Isaac Sim)
- Real robot control (Jetson Orin with hardware drivers)

The interface is simple: send velocity commands [vx, vy, omega] and
the controller handles the rest.
"""

import numpy as np
from abc import ABC, abstractmethod
from typing import List, Optional


class MotionController(ABC):
    """
    Abstract base class for robot motion control.

    Implement this interface for different backends:
    - SimulationMotionController: For Isaac Sim
    - RealRobotMotionController: For physical Jetson Orin hardware
    """

    @abstractmethod
    def set_velocity(self, vx: float, vy: float, omega: float) -> None:
        """
        Set the robot's desired velocity.

        Args:
            vx: Forward velocity (m/s) - positive = forward
            vy: Lateral velocity (m/s) - positive = left
            omega: Angular velocity (rad/s) - positive = counter-clockwise
        """
        pass

    @abstractmethod
    def reset(self) -> None:
        """Reset the controller state"""
        pass

    @abstractmethod
    def get_actual_velocity(self) -> tuple:
        """
        Get the robot's actual velocity (if available)

        Returns:
            tuple: (vx, vy, omega) actual velocities
        """
        pass


class MecanumKinematics:
    """
    Mecanum wheel kinematics calculator for X-configuration.

    This is pure mathematics - works for both simulation and real hardware.

    Wheel configuration (top view):
        Front
          ^
          |
    [FL] [FR]    FL: Front Left (-45° rollers)
                 FR: Front Right (+45° rollers)
    [RL] [RR]    RL: Rear Left (+45° rollers)
                 RR: Rear Right (-45° rollers)

    Coordinate system:
        X: Forward (positive = robot moves forward)
        Y: Left (positive = robot moves left)
        Z: Up
        Omega: Yaw rotation (positive = counter-clockwise)
    """

    def __init__(self,
                 wheel_base: float = 0.30,      # Distance between front and rear wheels (m)
                 track_width: float = 0.34,     # Distance between left and right wheels (m)
                 wheel_radius: float = 0.05):   # Wheel radius (m)
        """
        Initialize mecanum kinematics.

        Args:
            wheel_base: Distance between front and rear axles (m)
            track_width: Distance between left and right wheels (m)
            wheel_radius: Radius of each wheel (m)
        """
        self.wheel_base = wheel_base
        self.track_width = track_width
        self.wheel_radius = wheel_radius

        # Lx = half of wheel_base, Ly = half of track_width
        self.lx = wheel_base / 2.0
        self.ly = track_width / 2.0

    def inverse_kinematics(self, vx: float, vy: float, omega: float) -> np.ndarray:
        """
        Convert robot velocity to wheel angular velocities.

        This is the core mecanum kinematics equation:

        [ω_FL]   [1  -1  -(lx+ly)]   [vx  ]
        [ω_FR] = [1   1   (lx+ly)] * [vy  ]
        [ω_RL]   [1   1  -(lx+ly)]   [omega]
        [ω_RR]   [1  -1   (lx+ly)]

        All divided by wheel_radius

        Args:
            vx: Forward velocity (m/s)
            vy: Lateral velocity (m/s)
            omega: Angular velocity (rad/s)

        Returns:
            np.ndarray: Wheel angular velocities [FL, FR, RL, RR] in rad/s
        """
        r = self.wheel_radius
        lx_ly = self.lx + self.ly

        # Mecanum wheel inverse kinematics matrix
        # Each wheel's velocity is a combination of forward, lateral, and rotation
        wheel_fl = (vx - vy - lx_ly * omega) / r
        wheel_fr = (vx + vy + lx_ly * omega) / r
        wheel_rl = (vx + vy - lx_ly * omega) / r
        wheel_rr = (vx - vy + lx_ly * omega) / r

        return np.array([wheel_fl, wheel_fr, wheel_rl, wheel_rr])

    def forward_kinematics(self, wheel_velocities: np.ndarray) -> tuple:
        """
        Convert wheel angular velocities to robot velocity.

        This is the inverse of inverse_kinematics (the forward problem).
        Useful for estimating actual robot velocity from wheel encoders.

        Args:
            wheel_velocities: [FL, FR, RL, RR] wheel angular velocities (rad/s)

        Returns:
            tuple: (vx, vy, omega) robot velocities
        """
        r = self.wheel_radius
        lx_ly = self.lx + self.ly

        w_fl, w_fr, w_rl, w_rr = wheel_velocities

        # Forward kinematics (averaging to handle noise)
        vx = r * (w_fl + w_fr + w_rl + w_rr) / 4.0
        vy = r * (-w_fl + w_fr + w_rl - w_rr) / 4.0
        omega = r * (-w_fl + w_fr - w_rl + w_rr) / (4.0 * lx_ly)

        return (vx, vy, omega)


class SimulationMotionController(MotionController):
    """
    Motion controller for Isaac Sim simulation.

    IMPORTANT: This uses DIRECT POSE MANIPULATION (bypassing wheel physics)

    Why we bypass physics:
    1. The URDF wheels are simple cylinders (not proper mecanum wheels with angled rollers)
    2. PhysX cannot simulate the lateral forces from mecanum rollers that don't exist
    3. Result: Strafing doesn't work, rotation is poor
    4. This makes it impossible to test navigation, mapping, or any holonomic motion

    Solution:
    - We directly manipulate the robot's base position/orientation
    - This allows testing of:
      * Navigation algorithms
      * Computer vision
      * Path planning
      * Sensor integration
      * Any high-level behaviors
    - The REAL robot (Jetson Orin) will use actual mecanum wheels and work properly

    The abstraction ensures your high-level code works identically in both cases!
    """

    def __init__(self, robot, kinematics: MecanumKinematics, use_physics_bypass: bool = True, damping: float = 0.92):
        """
        Initialize simulation controller.

        Args:
            robot: Isaac Sim Robot instance (from isaacsim.core.api.robots)
            kinematics: MecanumKinematics instance
            use_physics_bypass: If True, directly move robot base (bypass wheel physics)
                               If False, try to use wheel physics (doesn't work well)
            damping: Friction coefficient (0-1). 0.92 means velocity decays by 8% per second.
                     Higher = more friction, lower = more slippery
        """
        self.robot = robot
        self.kinematics = kinematics
        self.current_command = np.array([0.0, 0.0, 0.0])  # [vx, vy, omega]
        self.actual_velocity = np.array([0.0, 0.0, 0.0])  # Current velocity (with damping)
        self.use_physics_bypass = use_physics_bypass
        self.damping = damping

        # For physics bypass mode
        self.last_update_time = None

        if self.use_physics_bypass:
            print("[SimulationMotionController] Using PHYSICS BYPASS mode")
            print("  Reason: URDF wheels are cylinders, not mecanum wheels")
            print("  Effect: Robot base will move directly (kinematically)")
            print("  Benefit: Navigation/mapping testing works correctly")
            print(f"  Damping: {damping} (simulated friction)")
            print("  Note: Real robot will use actual wheel physics")
        else:
            print("[SimulationMotionController] Using WHEEL PHYSICS mode")
            print("  Warning: Strafing will not work properly!")

    def set_velocity(self, vx: float, vy: float, omega: float) -> None:
        """
        Set robot velocity.

        In physics bypass mode: Directly updates robot pose each simulation step
        In wheel physics mode: Commands wheel velocities (doesn't work well)

        Args:
            vx: Forward velocity (m/s)
            vy: Lateral velocity (m/s)
            omega: Angular velocity (rad/s)
        """
        self.current_command = np.array([vx, vy, omega])

        if not self.use_physics_bypass:
            # Try to use wheel physics (doesn't work well for strafing)
            wheel_velocities = self.kinematics.inverse_kinematics(vx, vy, omega)
            self.robot.set_joint_velocities(wheel_velocities)
            # Note: Still spin wheels visually even in bypass mode
            # (This happens automatically in update() method)

    def update(self, dt: float) -> None:
        """
        Update robot pose based on commanded velocity.

        Call this every simulation step when using physics bypass mode.

        Args:
            dt: Time step (seconds)
        """
        if not self.use_physics_bypass:
            return  # Using wheel physics instead

        # Apply acceleration towards commanded velocity (smooth response)
        # This simulates motor acceleration and prevents instant velocity changes
        acceleration_factor = 10.0  # How fast we reach commanded velocity (higher = more responsive)
        for i in range(3):
            velocity_error = self.current_command[i] - self.actual_velocity[i]
            self.actual_velocity[i] += velocity_error * acceleration_factor * dt

        # Apply damping (friction) to simulate realistic deceleration
        # When command is zero, velocity smoothly decays to zero
        damping_factor = self.damping ** (dt * 60.0)  # Adjust for frame rate
        self.actual_velocity *= damping_factor

        # Stop completely if velocity is very small (prevent drift)
        if np.linalg.norm(self.actual_velocity) < 0.001:
            self.actual_velocity = np.array([0.0, 0.0, 0.0])

        vx, vy, omega = self.actual_velocity

        # Don't update pose if not moving
        if abs(vx) < 1e-6 and abs(vy) < 1e-6 and abs(omega) < 1e-6:
            # Still update wheel velocities to zero for visual feedback
            self.robot.set_joint_velocities(np.array([0.0, 0.0, 0.0, 0.0]))
            return

        # Get current pose
        position, orientation = self.robot.get_world_pose()

        # Convert quaternion to yaw angle
        # Quaternion format: [w, x, y, z] in Isaac Sim
        w, x, y, z = orientation
        yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

        # Transform velocities from robot frame to world frame
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)

        world_vx = vx * cos_yaw - vy * sin_yaw
        world_vy = vx * sin_yaw + vy * cos_yaw

        # Update position
        new_position = position + np.array([world_vx * dt, world_vy * dt, 0.0])

        # Update orientation
        new_yaw = yaw + omega * dt
        new_orientation = np.array([
            np.cos(new_yaw / 2.0),  # w
            0.0,                     # x
            0.0,                     # y
            np.sin(new_yaw / 2.0)   # z
        ])

        # Apply new pose
        self.robot.set_world_pose(position=new_position, orientation=new_orientation)

        # Also spin the wheels visually for realism
        wheel_velocities = self.kinematics.inverse_kinematics(vx, vy, omega)
        self.robot.set_joint_velocities(wheel_velocities)

    def reset(self) -> None:
        """Reset controller to stopped state"""
        self.current_command = np.array([0.0, 0.0, 0.0])
        self.actual_velocity = np.array([0.0, 0.0, 0.0])
        self.robot.set_joint_velocities(np.array([0.0, 0.0, 0.0, 0.0]))
        self.last_update_time = None

    def get_actual_velocity(self) -> tuple:
        """
        Get actual robot velocity.

        In physics bypass mode: Returns actual velocity (with damping applied)
        In wheel physics mode: Estimates from wheel velocities

        Returns:
            tuple: (vx, vy, omega)
        """
        if self.use_physics_bypass:
            # In bypass mode, return actual velocity (includes damping/acceleration)
            return tuple(self.actual_velocity)
        else:
            # In physics mode, estimate from wheels
            wheel_velocities = self.robot.get_joint_velocities()
            return self.kinematics.forward_kinematics(wheel_velocities)


class RealRobotMotionController(MotionController):
    """
    Motion controller for real Lukebot hardware (Jetson Orin).

    This is a STUB - implement this when deploying to real hardware.
    You would replace the simulation-specific code with:
    - ROS 2 topic publishing
    - Hardware driver calls
    - CAN bus commands
    - etc.
    """

    def __init__(self, kinematics: MecanumKinematics):
        """
        Initialize real robot controller.

        Args:
            kinematics: MecanumKinematics instance
        """
        self.kinematics = kinematics
        self.current_command = np.array([0.0, 0.0, 0.0])

        # TODO: Initialize hardware interfaces
        # - ROS 2 publishers/subscribers
        # - Motor controllers
        # - Wheel encoders
        pass

    def set_velocity(self, vx: float, vy: float, omega: float) -> None:
        """
        Set robot velocity on real hardware.

        Example implementation:
        ```python
        wheel_velocities = self.kinematics.inverse_kinematics(vx, vy, omega)

        # Publish to ROS 2
        self.wheel_velocity_publisher.publish(wheel_velocities)

        # OR send to motor controllers directly
        self.motor_controller.set_wheel_speeds(wheel_velocities)
        ```
        """
        self.current_command = np.array([vx, vy, omega])
        wheel_velocities = self.kinematics.inverse_kinematics(vx, vy, omega)

        # TODO: Implement hardware control
        # For now, just print what we would send
        print(f"[RealRobot] Would command wheels: {wheel_velocities}")

    def reset(self) -> None:
        """Stop the robot"""
        self.current_command = np.array([0.0, 0.0, 0.0])
        # TODO: Send stop command to hardware
        pass

    def get_actual_velocity(self) -> tuple:
        """
        Get actual velocity from wheel encoders.

        TODO: Read from hardware encoders
        """
        # For now, return commanded velocity
        return tuple(self.current_command)


# Factory function for easy controller creation
def create_motion_controller(robot=None,
                             use_simulation: bool = True,
                             use_physics_bypass: bool = True,
                             wheel_base: float = 0.30,
                             track_width: float = 0.34,
                             wheel_radius: float = 0.05,
                             damping: float = 0.92) -> MotionController:
    """
    Factory function to create the appropriate motion controller.

    Args:
        robot: Isaac Sim Robot instance (required if use_simulation=True)
        use_simulation: True for Isaac Sim, False for real hardware
        use_physics_bypass: If True (default), bypass wheel physics in simulation
                           (See SimulationMotionController docstring for why)
        wheel_base: Distance between front and rear wheels (m)
        track_width: Distance between left and right wheels (m)
        wheel_radius: Wheel radius (m)
        damping: Friction coefficient (0-1) for simulation. 0.92 = moderate friction.
                 Higher values = more friction/faster stopping

    Returns:
        MotionController: Either SimulationMotionController or RealRobotMotionController

    Example:
        # In Isaac Sim with physics bypass (recommended for testing)
        controller = create_motion_controller(
            robot=my_robot,
            use_simulation=True,
            use_physics_bypass=True  # Bypasses broken wheel physics
        )

        # On Jetson Orin (real hardware)
        controller = create_motion_controller(use_simulation=False)

        # Then use the same interface
        controller.set_velocity(vx=0.5, vy=0.0, omega=0.0)  # Move forward

        # In simulation loop, call update() each step
        if use_simulation:
            controller.update(dt=1.0/60.0)  # 60 FPS
    """
    kinematics = MecanumKinematics(
        wheel_base=wheel_base,
        track_width=track_width,
        wheel_radius=wheel_radius
    )

    if use_simulation:
        if robot is None:
            raise ValueError("robot parameter required for simulation mode")
        return SimulationMotionController(robot, kinematics, use_physics_bypass=use_physics_bypass, damping=damping)
    else:
        return RealRobotMotionController(kinematics)
