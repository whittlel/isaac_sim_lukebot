# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Lukebot Keyboard Teleoperation - MANUAL CONTROLLER PARAMETERS
Manually specify all HolonomicController parameters based on URDF geometry

This bypasses the automatic USD parameter extraction which may be buggy.

Controls:
  W - Move forward
  S - Move backward
  A - Strafe left
  D - Strafe right
  Q - Rotate counter-clockwise
  E - Rotate clockwise
  SPACE - Stop
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
from isaacsim.robot.wheeled_robots.controllers.holonomic_controller import HolonomicController
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from pxr import Gf, Sdf, UsdGeom, UsdPhysics

# Enable necessary extensions
enable_extension("omni.isaac.sensor")


class KeyboardController:
    """Simple keyboard controller for WASD movement with smooth acceleration"""

    def __init__(self):
        self.forward_speed = 0.0
        self.lateral_speed = 0.0
        self.rotation_speed = 0.0
        self.target_forward = 0.0
        self.target_lateral = 0.0
        self.target_rotation = 0.0
        self.max_linear_speed = 0.3  # m/s (reduced for stability)
        self.max_angular_speed = 0.5  # rad/s (reduced for stability)
        self.acceleration = 0.1  # Smooth acceleration factor

        # Get keyboard interface
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()
        self._sub_keyboard = self._input.subscribe_to_keyboard_events(self._keyboard, self._on_keyboard_event)

    def _on_keyboard_event(self, event, *args, **kwargs):
        """Handle keyboard events - set targets, not direct speeds"""
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
        """Smooth acceleration towards target speeds"""
        self.forward_speed += (self.target_forward - self.forward_speed) * self.acceleration
        self.lateral_speed += (self.target_lateral - self.lateral_speed) * self.acceleration
        self.rotation_speed += (self.target_rotation - self.rotation_speed) * self.acceleration

    def get_command(self):
        """Get current movement command [forward, lateral, rotation]"""
        return [self.forward_speed, self.lateral_speed, self.rotation_speed]

    def shutdown(self):
        """Clean up keyboard subscription"""
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
    carb.log_info("LUKEBOT TELEOPERATION - MANUAL CONTROLLER PARAMETERS")
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

    # Wheel joint names (ORDER MATTERS!)
    wheel_dof_names = [
        "front_left_wheel_joint",
        "front_right_wheel_joint",
        "rear_left_wheel_joint",
        "rear_right_wheel_joint",
    ]

    # MANUAL CONTROLLER PARAMETERS (from URDF analysis)
    # ================================================

    # Wheel radius - must be array with one value per wheel
    wheel_radius = np.array([0.050, 0.050, 0.050, 0.050])  # 50mm from URDF, same for all 4 wheels

    # Wheel positions from URDF (in robot frame: X-forward, Y-left, Z-up)
    # FL: xyz="0.15  0.17 -0.05"
    # FR: xyz="0.15 -0.17 -0.05"
    # RL: xyz="-0.15  0.17 -0.05"
    # RR: xyz="-0.15 -0.17 -0.05"
    wheel_positions = np.array([
        [0.15, 0.17, -0.05],   # Front Left
        [0.15, -0.17, -0.05],  # Front Right
        [-0.15, 0.17, -0.05],  # Rear Left
        [-0.15, -0.17, -0.05], # Rear Right
    ])

    # Wheel orientations (quaternions: w, x, y, z)
    # All wheels have same orientation (spinning around Y-axis)
    wheel_orientations = np.array([
        [1.0, 0.0, 0.0, 0.0],  # Front Left (identity quaternion)
        [1.0, 0.0, 0.0, 0.0],  # Front Right
        [1.0, 0.0, 0.0, 0.0],  # Rear Left
        [1.0, 0.0, 0.0, 0.0],  # Rear Right
    ])

    # Mecanum angles (radians) - INVERTED X-pattern (to fix strafe)
    # Standard: FL=+45°, FR=-45°, RL=-45°, RR=+45°
    # Inverted: FL=-45°, FR=+45°, RL=+45°, RR=-45°
    mecanum_angles = np.array([
        -np.pi / 4,  # Front Left: -45° (INVERTED)
        np.pi / 4,   # Front Right: +45° (INVERTED)
        np.pi / 4,   # Rear Left: +45° (INVERTED)
        -np.pi / 4,  # Rear Right: -45° (INVERTED)
    ])

    # Wheel axis (Y-axis from URDF: axis="0 1 0")
    wheel_axis = np.array([0.0, 1.0, 0.0])

    # Up axis (Z-axis, standard)
    up_axis = np.array([0.0, 0.0, 1.0])

    # Add mecanum wheel attributes to joints in USD
    carb.log_info("=" * 80)
    carb.log_info("CONFIGURING MECANUM WHEELS IN USD")
    carb.log_info("=" * 80)
    for i, (joint_name, angle) in enumerate(zip(wheel_dof_names, mecanum_angles)):
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
            # Add custom mecanum wheel attributes
            if not joint_prim.HasAttribute("isaacmecanumwheel:radius"):
                joint_prim.CreateAttribute("isaacmecanumwheel:radius", Sdf.ValueTypeNames.Float).Set(wheel_radius[i])
            else:
                joint_prim.GetAttribute("isaacmecanumwheel:radius").Set(wheel_radius[i])

            if not joint_prim.HasAttribute("isaacmecanumwheel:angle"):
                joint_prim.CreateAttribute("isaacmecanumwheel:angle", Sdf.ValueTypeNames.Float).Set(angle)
            else:
                joint_prim.GetAttribute("isaacmecanumwheel:angle").Set(angle)

            # Physics tuning for smooth control
            drive_api = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
            drive_api.GetDampingAttr().Set(1000.0)  # Very high damping for stability
            drive_api.GetStiffnessAttr().Set(0.0)
            drive_api.GetMaxForceAttr().Set(10000.0)  # High max force to overcome damping

            angle_deg = np.degrees(angle)
            carb.log_info(f"  [{i}] {joint_name}: {angle_deg:+6.1f}°")
        else:
            carb.log_warn(f"  ✗ Joint not found: {joint_name}")

    # Create test scene
    carb.log_info("Creating test scene...")

    # Forward marker (Red)
    forward_marker = stage.DefinePrim("/World/Markers/ForwardMarker", "Cube")
    UsdGeom.Xform(forward_marker).AddTranslateOp().Set(Gf.Vec3f(1.5, 0.0, 0.05))
    UsdGeom.Xform(forward_marker).AddScaleOp().Set(Gf.Vec3f(0.4, 0.1, 0.1))
    forward_marker.GetAttribute("primvars:displayColor").Set([(1.0, 0.0, 0.0)])

    # Left marker (Green)
    left_marker = stage.DefinePrim("/World/Markers/LeftMarker", "Cube")
    UsdGeom.Xform(left_marker).AddTranslateOp().Set(Gf.Vec3f(0.0, 1.5, 0.05))
    UsdGeom.Xform(left_marker).AddScaleOp().Set(Gf.Vec3f(0.1, 0.4, 0.1))
    left_marker.GetAttribute("primvars:displayColor").Set([(0.0, 1.0, 0.0)])

    # Target cube (Yellow)
    target_cube = stage.DefinePrim("/World/Obstacles/TargetCube", "Cube")
    UsdGeom.Xform(target_cube).AddTranslateOp().Set(Gf.Vec3f(2.5, 2.0, 0.25))
    UsdGeom.Xform(target_cube).AddScaleOp().Set(Gf.Vec3f(0.5, 0.5, 0.5))
    target_cube.GetAttribute("primvars:displayColor").Set([(1.0, 1.0, 0.0)])
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

    # Create HolonomicController with MANUAL parameters
    carb.log_info("=" * 80)
    carb.log_info("CREATING HOLONOMIC CONTROLLER (MANUAL PARAMETERS)")
    carb.log_info("=" * 80)
    carb.log_info(f"Wheel radii: {wheel_radius} m")
    carb.log_info(f"Wheel axis: {wheel_axis}")
    carb.log_info(f"Up axis: {up_axis}")
    carb.log_info("")
    carb.log_info("Wheel positions:")
    for i, pos in enumerate(wheel_positions):
        carb.log_info(f"  [{i}] {wheel_dof_names[i]}: ({pos[0]:+.2f}, {pos[1]:+.2f}, {pos[2]:+.2f})")
    carb.log_info("")
    carb.log_info("Mecanum angles:")
    for i, angle in enumerate(mecanum_angles):
        carb.log_info(f"  [{i}] {wheel_dof_names[i]}: {np.degrees(angle):+6.1f}°")
    carb.log_info("=" * 80)

    my_controller = HolonomicController(
        name="holonomic_controller",
        wheel_radius=wheel_radius,
        wheel_positions=wheel_positions,
        wheel_orientations=wheel_orientations,
        mecanum_angles=mecanum_angles,
        wheel_axis=wheel_axis,
        up_axis=up_axis,
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
    carb.log_info("Using MANUAL controller parameters from URDF")
    carb.log_info("No axis swapping - testing direct control")
    carb.log_info("=" * 80)

    step_count = 0
    reset_needed = False
    last_position = None

    try:
        while simulation_app.is_running():
            my_world.step(render=True)

            if my_world.is_stopped() and not reset_needed:
                reset_needed = True

            if my_world.is_playing():
                if reset_needed:
                    my_world.reset()
                    my_controller.reset()
                    reset_needed = False
                    step_count = 0
                    last_position = None

                # Update keyboard controller with smooth acceleration
                keyboard_ctrl.update()

                # Get keyboard command
                command = keyboard_ctrl.get_command()

                # COMPREHENSIVE FIX based on testing:
                # - First axis swap worked for W/S: [lateral, forward, rotation]
                # - But lateral (A/D) isn't working, needs negation
                # Original: [forward, lateral, rotation]
                # Test:     [lateral, -forward, -rotation]
                # If W/S inverted, remove the negation on forward
                command_to_apply = [command[1], command[0], -command[2]]

                if my_lukebot:
                    wheel_actions = my_controller.forward(command=command_to_apply)

                    # Log wheel actions on every frame when there's active input
                    if any(c != 0 for c in command):
                        if step_count % 10 == 0:  # Log every 10 frames to avoid spam
                            carb.log_info(f"[CMD] Input: F={command[0]:.2f} L={command[1]:.2f} R={command[2]:.2f} → "
                                        f"Applied: F={command_to_apply[0]:.2f} L={command_to_apply[1]:.2f} R={command_to_apply[2]:.2f}")
                            if wheel_actions.joint_velocities is not None and len(wheel_actions.joint_velocities) >= 4:
                                carb.log_info(f"[WHL] Commanded: FL={wheel_actions.joint_velocities[0]:+.2f} "
                                            f"FR={wheel_actions.joint_velocities[1]:+.2f} "
                                            f"RL={wheel_actions.joint_velocities[2]:+.2f} "
                                            f"RR={wheel_actions.joint_velocities[3]:+.2f} rad/s")

                    my_lukebot.apply_wheel_actions(wheel_actions)

                    # Diagnostics
                    if step_count % 60 == 0 and step_count > 0 and any(c != 0 for c in command):
                        current_position, _ = my_lukebot.get_world_pose()
                        if last_position is not None:
                            delta_pos = current_position - last_position
                            delta_magnitude = np.linalg.norm(delta_pos[:2])
                            if delta_magnitude > 0.005:
                                direction = np.arctan2(delta_pos[1], delta_pos[0])
                                direction_deg = np.degrees(direction)

                                status = ""
                                if command[0] != 0:  # Forward/back command
                                    if abs(direction_deg) < 20 or abs(abs(direction_deg) - 180) < 20:
                                        status = "✓ FORWARD/BACK OK"
                                    else:
                                        status = f"✗ Should be 0° or 180°, got {direction_deg:.0f}°"
                                elif command[1] != 0:  # Lateral command
                                    if abs(abs(direction_deg) - 90) < 20:
                                        status = "✓ LATERAL OK"
                                    else:
                                        status = f"✗ Should be ±90°, got {direction_deg:.0f}°"

                                carb.log_info(f"[{step_count:4d}] Pos: ({current_position[0]:+.2f}, {current_position[1]:+.2f}) | "
                                            f"Dir: {direction_deg:+4.0f}° | Cmd: F={command[0]:.1f} L={command[1]:.1f} | {status}")

                                # Also log wheel velocities for diagnosis
                                wheel_vels = my_lukebot.get_joint_velocities()
                                if wheel_vels is not None and len(wheel_vels) == 4:
                                    carb.log_info(f"       Wheels: FL={wheel_vels[0]:+.2f} FR={wheel_vels[1]:+.2f} "
                                                f"RL={wheel_vels[2]:+.2f} RR={wheel_vels[3]:+.2f} rad/s")

                        last_position = current_position.copy()

                step_count += 1

    finally:
        keyboard_ctrl.shutdown()
        simulation_app.close()


if __name__ == "__main__":
    main()
