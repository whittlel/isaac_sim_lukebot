# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Lukebot Keyboard Teleoperation - CUSTOM CONTROLLER
Bypasses HolonomicController with direct wheel velocity calculation

This implements mecanum wheel kinematics manually for full control.
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import carb
import numpy as np
import omni.appwindow
import omni.kit.commands
from isaacsim.core.api import World
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import get_current_stage
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from isaacsim.core.utils.types import ArticulationAction
from pxr import Gf, Sdf, UsdGeom, UsdPhysics

# Enable necessary extensions
enable_extension("omni.isaac.sensor")


class MecanumController:
    """
    Custom Mecanum Wheel Controller

    Implements inverse kinematics manually for precise control.
    """

    def __init__(self, wheel_radius=0.05, wheelbase_length=0.30, wheelbase_width=0.34):
        """
        Args:
            wheel_radius: Radius of wheels in meters
            wheelbase_length: Distance between front and rear wheels (X-axis)
            wheelbase_width: Distance between left and right wheels (Y-axis)
        """
        self.wheel_radius = wheel_radius
        self.lx = wheelbase_length / 2.0  # Half of wheelbase length
        self.ly = wheelbase_width / 2.0   # Half of wheelbase width

        carb.log_info(f"MecanumController initialized:")
        carb.log_info(f"  Wheel radius: {wheel_radius}m")
        carb.log_info(f"  Wheelbase: L={wheelbase_length}m, W={wheelbase_width}m")

    def calculate_wheel_velocities(self, vx, vy, omega):
        """
        Calculate wheel velocities from robot velocities

        Args:
            vx: Forward velocity (m/s)
            vy: Lateral velocity (m/s, positive = left)
            omega: Rotational velocity (rad/s, positive = CCW)

        Returns:
            Array of wheel velocities [FL, FR, RL, RR] in rad/s
        """
        # Mecanum wheel inverse kinematics
        # Standard X-pattern configuration
        # FL = +45°, FR = -45°, RL = -45°, RR = +45°

        # Calculate wheel velocities (rad/s)
        # These equations come from mecanum kinematics
        fl = (vx - vy - (self.lx + self.ly) * omega) / self.wheel_radius
        fr = (vx + vy + (self.lx + self.ly) * omega) / self.wheel_radius
        rl = (vx + vy - (self.lx + self.ly) * omega) / self.wheel_radius
        rr = (vx - vy + (self.lx + self.ly) * omega) / self.wheel_radius

        return np.array([fl, fr, rl, rr])


class KeyboardController:
    """Keyboard controller with smooth acceleration"""

    def __init__(self):
        self.forward_speed = 0.0
        self.lateral_speed = 0.0
        self.rotation_speed = 0.0
        self.target_forward = 0.0
        self.target_lateral = 0.0
        self.target_rotation = 0.0
        self.max_linear_speed = 0.5  # m/s
        self.max_angular_speed = 1.0  # rad/s
        self.acceleration = 0.15  # Smooth acceleration

        # Get keyboard interface
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()
        self._sub_keyboard = self._input.subscribe_to_keyboard_events(self._keyboard, self._on_keyboard_event)

        carb.log_info("Keyboard controller initialized")

    def _on_keyboard_event(self, event, *args, **kwargs):
        """Handle keyboard events"""
        if event.type == carb.input.KeyboardEventType.KEY_PRESS or event.type == carb.input.KeyboardEventType.KEY_REPEAT:
            if event.input == carb.input.KeyboardInput.W:
                self.target_forward = self.max_linear_speed
            elif event.input == carb.input.KeyboardInput.S:
                self.target_forward = -self.max_linear_speed
            elif event.input == carb.input.KeyboardInput.A:
                self.target_lateral = self.max_linear_speed
            elif event.input == carb.input.KeyboardInput.D:
                self.target_lateral = -self.max_linear_speed
            elif event.input == carb.input.KeyboardInput.Q:
                self.target_rotation = self.max_angular_speed
            elif event.input == carb.input.KeyboardInput.E:
                self.target_rotation = -self.max_angular_speed
            elif event.input == carb.input.KeyboardInput.SPACE:
                self.target_forward = 0.0
                self.target_lateral = 0.0
                self.target_rotation = 0.0

        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            if event.input == carb.input.KeyboardInput.W or event.input == carb.input.KeyboardInput.S:
                self.target_forward = 0.0
            elif event.input == carb.input.KeyboardInput.A or event.input == carb.input.KeyboardInput.D:
                self.target_lateral = 0.0
            elif event.input == carb.input.KeyboardInput.Q or event.input == carb.input.KeyboardInput.E:
                self.target_rotation = 0.0

        return True

    def update(self):
        """Smooth acceleration"""
        self.forward_speed += (self.target_forward - self.forward_speed) * self.acceleration
        self.lateral_speed += (self.target_lateral - self.lateral_speed) * self.acceleration
        self.rotation_speed += (self.target_rotation - self.rotation_speed) * self.acceleration

    def get_command(self):
        """Get current velocities [vx, vy, omega]"""
        return [self.forward_speed, self.lateral_speed, self.rotation_speed]

    def shutdown(self):
        """Clean up"""
        if self._sub_keyboard:
            self._input.unsubscribe_from_keyboard_events(self._keyboard, self._sub_keyboard)
            self._sub_keyboard = None


def main():
    # Create world
    my_world = World(stage_units_in_meters=1.0)
    my_world.scene.add_default_ground_plane()

    # Set camera view
    set_camera_view(eye=[3.0, 3.0, 2.5], target=[0.0, 0.0, 0.3], camera_prim_path="/OmniverseKit_Persp")

    # Get URDF path
    urdf_path = "C:/Users/Luke/Desktop/issac_sim/source/extensions/isaacsim.asset.importer.urdf/data/urdf/robots/lukebot/urdf/lukebot.urdf"

    carb.log_info("=" * 80)
    carb.log_info("LUKEBOT TELEOPERATION - CUSTOM CONTROLLER")
    carb.log_info("=" * 80)
    carb.log_info("Importing Lukebot URDF...")

    # Create import configuration
    status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    import_config.merge_fixed_joints = False
    import_config.convex_decomp = False
    import_config.import_inertia_tensor = True
    import_config.fix_base = False
    import_config.make_default_prim = False
    import_config.self_collision = False
    import_config.create_physics_scene = False
    import_config.distance_scale = 1.0

    # Import URDF
    omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=urdf_path,
        import_config=import_config,
    )

    stage = get_current_stage()

    # Find the imported robot
    robot_prim = None
    for prim in stage.Traverse():
        if prim.GetName() == "lukebot":
            robot_prim = prim
            robot_prim_path = str(prim.GetPath())
            carb.log_info(f"Found lukebot at: {robot_prim_path}")
            break

    if robot_prim is None or not robot_prim.IsValid():
        carb.log_error("Failed to find lukebot robot after import")
        simulation_app.close()
        return

    # Wheel joint names
    wheel_dof_names = [
        "front_left_wheel_joint",
        "front_right_wheel_joint",
        "rear_left_wheel_joint",
        "rear_right_wheel_joint",
    ]

    # Configure wheel physics for smooth operation
    # Also add mecanum wheel metadata for physics engine
    carb.log_info("Configuring wheel physics...")

    wheel_radius = 0.05
    # Standard X-pattern
    mecanum_angles_deg = [45, -45, -45, 45]

    for i, joint_name in enumerate(wheel_dof_names):
        possible_paths = [
            f"{robot_prim_path}/{joint_name}",
            f"{robot_prim_path}/joints/{joint_name}",
        ]

        joint_prim = None
        for joint_path in possible_paths:
            joint_prim = stage.GetPrimAtPath(joint_path)
            if joint_prim.IsValid():
                break

        if joint_prim and joint_prim.IsValid():
            # Add mecanum wheel attributes for physics
            angle_rad = np.radians(mecanum_angles_deg[i])

            if not joint_prim.HasAttribute("isaacmecanumwheel:radius"):
                joint_prim.CreateAttribute("isaacmecanumwheel:radius", Sdf.ValueTypeNames.Float).Set(wheel_radius)
            else:
                joint_prim.GetAttribute("isaacmecanumwheel:radius").Set(wheel_radius)

            if not joint_prim.HasAttribute("isaacmecanumwheel:angle"):
                joint_prim.CreateAttribute("isaacmecanumwheel:angle", Sdf.ValueTypeNames.Float).Set(angle_rad)
            else:
                joint_prim.GetAttribute("isaacmecanumwheel:angle").Set(angle_rad)

            # Configure drive for velocity control
            drive_api = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
            drive_api.GetDampingAttr().Set(100.0)  # Moderate damping
            drive_api.GetStiffnessAttr().Set(0.0)   # No stiffness (velocity control)
            drive_api.GetMaxForceAttr().Set(1000.0) # Reasonable max force
            carb.log_info(f"  ✓ {joint_name}: angle={mecanum_angles_deg[i]:+.0f}°")

    # Visual reference markers
    forward_marker = stage.DefinePrim("/World/Markers/ForwardMarker", "Cube")
    UsdGeom.Xform(forward_marker).AddTranslateOp().Set(Gf.Vec3f(1.5, 0.0, 0.05))
    UsdGeom.Xform(forward_marker).AddScaleOp().Set(Gf.Vec3f(0.4, 0.1, 0.1))
    forward_marker.GetAttribute("primvars:displayColor").Set([(1.0, 0.0, 0.0)])  # Red = Forward

    left_marker = stage.DefinePrim("/World/Markers/LeftMarker", "Cube")
    UsdGeom.Xform(left_marker).AddTranslateOp().Set(Gf.Vec3f(0.0, 1.5, 0.05))
    UsdGeom.Xform(left_marker).AddScaleOp().Set(Gf.Vec3f(0.1, 0.4, 0.1))
    left_marker.GetAttribute("primvars:displayColor").Set([(0.0, 1.0, 0.0)])  # Green = Left

    target_cube = stage.DefinePrim("/World/Obstacles/TargetCube", "Cube")
    UsdGeom.Xform(target_cube).AddTranslateOp().Set(Gf.Vec3f(2.5, 2.0, 0.25))
    UsdGeom.Xform(target_cube).AddScaleOp().Set(Gf.Vec3f(0.5, 0.5, 0.5))
    target_cube.GetAttribute("primvars:displayColor").Set([(1.0, 1.0, 0.0)])  # Yellow
    UsdPhysics.CollisionAPI.Apply(target_cube)
    UsdPhysics.RigidBodyAPI.Apply(target_cube)

    # Add Lukebot as a WheeledRobot
    my_lukebot = my_world.scene.add(
        WheeledRobot(
            prim_path=robot_prim_path,
            name="my_lukebot",
            wheel_dof_names=wheel_dof_names,
            create_robot=False,
            position=np.array([0, 0.0, 0.1]),
        )
    )

    # Create CUSTOM mecanum controller
    carb.log_info("Creating custom mecanum controller...")
    # From URDF: wheelbase_length = 0.30m (front to back), wheelbase_width = 0.34m (left to right)
    my_controller = MecanumController(
        wheel_radius=0.05,
        wheelbase_length=0.30,
        wheelbase_width=0.34
    )

    # Initialize keyboard controller
    keyboard_ctrl = KeyboardController()

    # Reset world
    my_world.reset()

    carb.log_info("=" * 80)
    carb.log_info("LUKEBOT TELEOPERATION READY!")
    carb.log_info("=" * 80)
    carb.log_info("CONTROLS:")
    carb.log_info("  W - Move forward (toward RED marker)")
    carb.log_info("  S - Move backward")
    carb.log_info("  A - Strafe left (toward GREEN marker)")
    carb.log_info("  D - Strafe right")
    carb.log_info("  Q - Rotate counter-clockwise")
    carb.log_info("  E - Rotate clockwise")
    carb.log_info("  SPACE - Stop")
    carb.log_info("")
    carb.log_info("Using CUSTOM mecanum controller with manual kinematics")
    carb.log_info("=" * 80)

    step_count = 0
    reset_needed = False

    try:
        while simulation_app.is_running():
            my_world.step(render=True)

            if my_world.is_stopped() and not reset_needed:
                reset_needed = True

            if my_world.is_playing():
                if reset_needed:
                    my_world.reset()
                    reset_needed = False
                    step_count = 0

                # Update keyboard with smooth acceleration
                keyboard_ctrl.update()

                # Get velocities [vx, vy, omega]
                vx, vy, omega = keyboard_ctrl.get_command()

                # Calculate wheel velocities using custom controller
                wheel_velocities = my_controller.calculate_wheel_velocities(vx, vy, omega)

                # Log occasionally
                if step_count % 120 == 0 and (abs(vx) > 0.01 or abs(vy) > 0.01 or abs(omega) > 0.01):
                    carb.log_info(f"Cmd: vx={vx:.2f} vy={vy:.2f} ω={omega:.2f} | "
                                f"Wheels: FL={wheel_velocities[0]:+.1f} FR={wheel_velocities[1]:+.1f} "
                                f"RL={wheel_velocities[2]:+.1f} RR={wheel_velocities[3]:+.1f} rad/s")

                # Apply wheel actions
                if my_lukebot:
                    action = ArticulationAction(joint_velocities=wheel_velocities)
                    my_lukebot.apply_wheel_actions(action)

                step_count += 1

    finally:
        keyboard_ctrl.shutdown()
        simulation_app.close()


if __name__ == "__main__":
    main()
