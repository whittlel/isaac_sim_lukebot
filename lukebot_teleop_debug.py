# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Lukebot Mecanum Wheel Diagnostic Tool
Debug mecanum wheel configuration with detailed telemetry

Controls:
  W - Move forward
  S - Move backward
  A - Strafe left
  D - Strafe right
  Q - Rotate counter-clockwise
  E - Rotate clockwise
  SPACE - Stop

  1 - Test front-left wheel only
  2 - Test front-right wheel only
  3 - Test rear-left wheel only
  4 - Test rear-right wheel only
  0 - Normal mode (all wheels)

  ESC - Exit
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
from isaacsim.robot.wheeled_robots.robots.holonomic_robot_usd_setup import HolonomicRobotUsdSetup
from pxr import Gf, Sdf, UsdGeom, UsdPhysics

# Enable necessary extensions
enable_extension("omni.isaac.sensor")


class KeyboardController:
    """Enhanced keyboard controller with wheel diagnostics"""

    def __init__(self):
        self.forward_speed = 0.0
        self.lateral_speed = 0.0
        self.rotation_speed = 0.0
        self.max_linear_speed = 0.3  # m/s (slower for debugging)
        self.max_angular_speed = 0.5  # rad/s (slower for debugging)
        self.test_mode = 0  # 0=normal, 1-4=individual wheels

        # Get keyboard interface
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()
        self._sub_keyboard = self._input.subscribe_to_keyboard_events(self._keyboard, self._on_keyboard_event)

        carb.log_info("=" * 80)
        carb.log_info("MECANUM WHEEL DIAGNOSTIC MODE")
        carb.log_info("=" * 80)
        carb.log_info("Controls:")
        carb.log_info("  W/S - Forward/Backward")
        carb.log_info("  A/D - Strafe Left/Right")
        carb.log_info("  Q/E - Rotate CCW/CW")
        carb.log_info("  SPACE - Stop")
        carb.log_info("")
        carb.log_info("Wheel Test Mode:")
        carb.log_info("  1 - Test FRONT-LEFT wheel only")
        carb.log_info("  2 - Test FRONT-RIGHT wheel only")
        carb.log_info("  3 - Test REAR-LEFT wheel only")
        carb.log_info("  4 - Test REAR-RIGHT wheel only")
        carb.log_info("  0 - Normal mode (all wheels)")
        carb.log_info("=" * 80)

    def _on_keyboard_event(self, event, *args, **kwargs):
        """Handle keyboard events"""
        if event.type == carb.input.KeyboardEventType.KEY_PRESS or event.type == carb.input.KeyboardEventType.KEY_REPEAT:
            # Forward/Backward (W/S)
            if event.input == carb.input.KeyboardInput.W:
                self.forward_speed = self.max_linear_speed
            elif event.input == carb.input.KeyboardInput.S:
                self.forward_speed = -self.max_linear_speed

            # Strafe Left/Right (A/D)
            if event.input == carb.input.KeyboardInput.A:
                self.lateral_speed = self.max_linear_speed
            elif event.input == carb.input.KeyboardInput.D:
                self.lateral_speed = -self.max_linear_speed

            # Rotate (Q/E)
            if event.input == carb.input.KeyboardInput.Q:
                self.rotation_speed = self.max_angular_speed
            elif event.input == carb.input.KeyboardInput.E:
                self.rotation_speed = -self.max_angular_speed

            # Stop (SPACE)
            if event.input == carb.input.KeyboardInput.SPACE:
                self.forward_speed = 0.0
                self.lateral_speed = 0.0
                self.rotation_speed = 0.0

            # Test modes (0-4)
            if event.input == carb.input.KeyboardInput.KEY_0:
                self.test_mode = 0
                carb.log_info(">> NORMAL MODE: All wheels active")
            elif event.input == carb.input.KeyboardInput.KEY_1:
                self.test_mode = 1
                carb.log_info(">> TEST MODE: Front-Left wheel only")
            elif event.input == carb.input.KeyboardInput.KEY_2:
                self.test_mode = 2
                carb.log_info(">> TEST MODE: Front-Right wheel only")
            elif event.input == carb.input.KeyboardInput.KEY_3:
                self.test_mode = 3
                carb.log_info(">> TEST MODE: Rear-Left wheel only")
            elif event.input == carb.input.KeyboardInput.KEY_4:
                self.test_mode = 4
                carb.log_info(">> TEST MODE: Rear-Right wheel only")

        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            # Stop movement when key is released
            if event.input == carb.input.KeyboardInput.W or event.input == carb.input.KeyboardInput.S:
                self.forward_speed = 0.0
            elif event.input == carb.input.KeyboardInput.A or event.input == carb.input.KeyboardInput.D:
                self.lateral_speed = 0.0
            elif event.input == carb.input.KeyboardInput.Q or event.input == carb.input.KeyboardInput.E:
                self.rotation_speed = 0.0

        return True

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
    set_camera_view(eye=[2.5, 2.5, 2.0], target=[0.0, 0.0, 0.3], camera_prim_path="/OmniverseKit_Persp")

    # Get URDF path
    urdf_path = "C:/Users/Luke/Desktop/issac_sim/source/extensions/isaacsim.asset.importer.urdf/data/urdf/robots/lukebot/urdf/lukebot.urdf"

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

    # Add mecanum wheel attributes to wheel joints
    wheel_radius = 0.050  # 50mm = 0.05m

    # TESTING: Let's try different mecanum angle configurations
    # Standard mecanum: FL=+45°, FR=-45°, RL=-45°, RR=+45°
    mecanum_angles = [np.pi / 4, -np.pi / 4, -np.pi / 4, np.pi / 4]  # radians

    carb.log_info("=" * 80)
    carb.log_info("CONFIGURING MECANUM WHEELS")
    carb.log_info("=" * 80)
    for i, (joint_name, angle) in enumerate(zip(wheel_dof_names, mecanum_angles)):
        # Try different possible paths where the joint might be
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
                joint_prim.CreateAttribute("isaacmecanumwheel:radius", Sdf.ValueTypeNames.Float).Set(wheel_radius)
            else:
                joint_prim.GetAttribute("isaacmecanumwheel:radius").Set(wheel_radius)

            if not joint_prim.HasAttribute("isaacmecanumwheel:angle"):
                joint_prim.CreateAttribute("isaacmecanumwheel:angle", Sdf.ValueTypeNames.Float).Set(angle)
            else:
                joint_prim.GetAttribute("isaacmecanumwheel:angle").Set(angle)

            # Set drive properties
            drive_api = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
            drive_api.GetDampingAttr().Set(1.0)
            drive_api.GetStiffnessAttr().Set(0.0)

            angle_deg = np.degrees(angle)
            carb.log_info(f"  [{i}] {joint_name}: angle={angle_deg:+.1f}°, radius={wheel_radius}m")
        else:
            carb.log_warn(f"  ✗ Joint not found: {joint_name}")

    carb.log_info("=" * 80)

    # Add visual reference grid
    carb.log_info("Creating reference markers...")

    # Forward direction marker (Red arrow)
    forward_marker = stage.DefinePrim("/World/Markers/ForwardMarker", "Cube")
    UsdGeom.Xform(forward_marker).AddTranslateOp().Set(Gf.Vec3f(1.0, 0.0, 0.05))
    UsdGeom.Xform(forward_marker).AddScaleOp().Set(Gf.Vec3f(0.3, 0.1, 0.1))
    forward_marker.GetAttribute("primvars:displayColor").Set([(1.0, 0.0, 0.0)])  # Red

    # Left direction marker (Green arrow)
    left_marker = stage.DefinePrim("/World/Markers/LeftMarker", "Cube")
    UsdGeom.Xform(left_marker).AddTranslateOp().Set(Gf.Vec3f(0.0, 1.0, 0.05))
    UsdGeom.Xform(left_marker).AddScaleOp().Set(Gf.Vec3f(0.1, 0.3, 0.1))
    left_marker.GetAttribute("primvars:displayColor").Set([(0.0, 1.0, 0.0)])  # Green

    # Add Lukebot as a WheeledRobot to the scene
    my_lukebot = my_world.scene.add(
        WheeledRobot(
            prim_path=robot_prim_path,
            name="my_lukebot",
            wheel_dof_names=wheel_dof_names,
            create_robot=False,  # Already imported from URDF
            position=np.array([0, 0.0, 0.1]),
        )
    )

    # Setup HolonomicController
    carb.log_info("Setting up HolonomicController...")
    lukebot_setup = HolonomicRobotUsdSetup(
        robot_prim_path=robot_prim_path, com_prim_path=f"{robot_prim_path}/chassis_link"
    )

    (
        wheel_radius_params,
        wheel_positions,
        wheel_orientations,
        mecanum_angles_params,
        wheel_axis,
        up_axis,
    ) = lukebot_setup.get_holonomic_controller_params()

    # DETAILED DEBUG OUTPUT
    carb.log_info("=" * 80)
    carb.log_info("HOLONOMIC CONTROLLER PARAMETERS")
    carb.log_info("=" * 80)
    carb.log_info(f"Wheel radius: {wheel_radius_params}")
    carb.log_info(f"Wheel axis: {wheel_axis}")
    carb.log_info(f"Up axis: {up_axis}")
    carb.log_info("")
    carb.log_info("Wheel positions (x, y, z):")
    for i, pos in enumerate(wheel_positions):
        carb.log_info(f"  [{i}] {wheel_dof_names[i]}: ({pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f})")
    carb.log_info("")
    carb.log_info("Wheel orientations (quaternion w, x, y, z):")
    for i, orient in enumerate(wheel_orientations):
        carb.log_info(f"  [{i}] {wheel_dof_names[i]}: ({orient[0]:+.3f}, {orient[1]:+.3f}, {orient[2]:+.3f}, {orient[3]:+.3f})")
    carb.log_info("")
    carb.log_info("Mecanum angles (radians):")
    for i, angle in enumerate(mecanum_angles_params):
        angle_deg = np.degrees(angle)
        carb.log_info(f"  [{i}] {wheel_dof_names[i]}: {angle:+.3f} rad ({angle_deg:+.1f}°)")
    carb.log_info("=" * 80)

    my_controller = HolonomicController(
        name="holonomic_controller",
        wheel_radius=wheel_radius_params,
        wheel_positions=wheel_positions,
        wheel_orientations=wheel_orientations,
        mecanum_angles=mecanum_angles_params,
        wheel_axis=wheel_axis,
        up_axis=up_axis,
    )

    # Initialize keyboard controller
    keyboard_ctrl = KeyboardController()

    # Reset world
    my_world.reset()

    carb.log_info("=" * 80)
    carb.log_info("DIAGNOSTIC MODE READY!")
    carb.log_info("=" * 80)
    carb.log_info("1. Try pressing 'W' to move forward")
    carb.log_info("2. Observe which direction the robot actually moves")
    carb.log_info("3. Try 'A' to strafe left and observe movement")
    carb.log_info("4. Use keys 1-4 to test individual wheels")
    carb.log_info("=" * 80)

    step_count = 0
    reset_needed = False
    last_command = [0.0, 0.0, 0.0]
    last_position = None
    last_orientation = None

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

                # Get keyboard command
                command = keyboard_ctrl.get_command()

                # Get robot state for diagnostics
                if my_lukebot:
                    current_position, current_orientation = my_lukebot.get_world_pose()

                    # Calculate position change
                    if last_position is not None and step_count % 60 == 0 and any(c != 0 for c in command):
                        delta_pos = current_position - last_position
                        delta_magnitude = np.linalg.norm(delta_pos[:2])  # Only XY
                        if delta_magnitude > 0.001:
                            direction = np.arctan2(delta_pos[1], delta_pos[0])
                            direction_deg = np.degrees(direction)
                            carb.log_info("=" * 60)
                            carb.log_info(f"[Step {step_count}] ROBOT TELEMETRY")
                            carb.log_info(f"  Command: Forward={command[0]:+.2f}, Lateral={command[1]:+.2f}, Rot={command[2]:+.2f}")
                            carb.log_info(f"  Position: ({current_position[0]:+.3f}, {current_position[1]:+.3f}, {current_position[2]:+.3f})")
                            carb.log_info(f"  Movement: ΔX={delta_pos[0]:+.4f}, ΔY={delta_pos[1]:+.4f}, magnitude={delta_magnitude:.4f}")
                            carb.log_info(f"  Direction: {direction_deg:+.1f}° (0°=+X, 90°=+Y)")

                            # Interpret movement
                            if command[0] > 0 and command[1] == 0 and command[2] == 0:
                                carb.log_info(f"  Expected: Forward (+X axis, 0°)")
                                if abs(direction_deg) < 30:
                                    carb.log_info(f"  ✓ CORRECT: Robot moving forward!")
                                else:
                                    carb.log_warn(f"  ✗ PROBLEM: Robot NOT moving forward (off by {abs(direction_deg):.1f}°)")
                            elif command[1] > 0 and command[0] == 0 and command[2] == 0:
                                carb.log_info(f"  Expected: Strafe left (+Y axis, 90°)")
                                if abs(direction_deg - 90) < 30:
                                    carb.log_info(f"  ✓ CORRECT: Robot strafing left!")
                                else:
                                    carb.log_warn(f"  ✗ PROBLEM: Robot NOT strafing left (off by {abs(direction_deg - 90):.1f}°)")

                            # Get wheel velocities for diagnosis
                            wheel_velocities = my_lukebot.get_joint_velocities()
                            if wheel_velocities is not None:
                                carb.log_info(f"  Wheel velocities:")
                                for i, vel in enumerate(wheel_velocities):
                                    carb.log_info(f"    [{i}] {wheel_dof_names[i]}: {vel:+.3f} rad/s")

                            carb.log_info("=" * 60)

                    last_position = current_position.copy()

                # Apply command with different strategies for testing
                # Try NO transformation first to see raw behavior
                command_to_apply = command  # Direct, no swap

                # In test mode, zero out other wheels
                if keyboard_ctrl.test_mode > 0:
                    wheel_actions = my_controller.forward(command=command_to_apply)
                    # Zero out all wheels except the test wheel
                    for i in range(len(wheel_actions.joint_velocities)):
                        if i != (keyboard_ctrl.test_mode - 1):
                            wheel_actions.joint_velocities[i] = 0.0
                    my_lukebot.apply_wheel_actions(wheel_actions)
                else:
                    # Normal mode - all wheels active
                    wheel_actions = my_controller.forward(command=command_to_apply)
                    my_lukebot.apply_wheel_actions(wheel_actions)

                # Log command changes
                if command != last_command and any(c != 0 for c in command):
                    carb.log_info(f"\n>> Command: Forward={command[0]:.2f}, Lateral={command[1]:.2f}, Rotation={command[2]:.2f}")
                    if keyboard_ctrl.test_mode == 0:
                        carb.log_info(f"   (All wheels active)")
                    last_command = command.copy()

                step_count += 1

    finally:
        # Clean up keyboard controller
        keyboard_ctrl.shutdown()
        simulation_app.close()


if __name__ == "__main__":
    main()
