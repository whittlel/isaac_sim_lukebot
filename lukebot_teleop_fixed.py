# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Lukebot Keyboard Teleoperation - FIXED VERSION
Fixed mecanum wheel configuration to resolve axis swap issue

Controls:
  W - Move forward
  S - Move backward
  A - Strafe left
  D - Strafe right
  Q - Rotate counter-clockwise
  E - Rotate clockwise
  SPACE - Stop
  ESC - Exit

FIXES APPLIED:
1. Swap forward/lateral in controller command (axes swapped in URDF)
2. Increase damping to reduce jitter (10.0 -> reduces wheel fighting)
3. Negate rotation direction for correct turning
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
    """Simple keyboard controller for WASD movement"""

    def __init__(self):
        self.forward_speed = 0.0
        self.lateral_speed = 0.0
        self.rotation_speed = 0.0
        self.max_linear_speed = 0.5  # m/s
        self.max_angular_speed = 1.0  # rad/s

        # Get keyboard interface
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()
        self._sub_keyboard = self._input.subscribe_to_keyboard_events(self._keyboard, self._on_keyboard_event)

        carb.log_info("Keyboard controller initialized")
        carb.log_info("Controls:")
        carb.log_info("  W/S - Forward/Backward")
        carb.log_info("  A/D - Strafe Left/Right")
        carb.log_info("  Q/E - Rotate CCW/CW")
        carb.log_info("  SPACE - Stop")

    def _on_keyboard_event(self, event, *args, **kwargs):
        """Handle keyboard events"""
        # Only process key press and release events
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
    set_camera_view(eye=[3.0, 3.0, 2.5], target=[0.0, 0.0, 0.3], camera_prim_path="/OmniverseKit_Persp")

    # Get URDF path
    urdf_path = "C:/Users/Luke/Desktop/issac_sim/source/extensions/isaacsim.asset.importer.urdf/data/urdf/robots/lukebot/urdf/lukebot.urdf"

    carb.log_info("=" * 80)
    carb.log_info("LUKEBOT KEYBOARD TELEOPERATION - FIXED VERSION")
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

    # Add mecanum wheel attributes to wheel joints
    wheel_radius = 0.050  # 50mm = 0.05m
    # Standard mecanum configuration: FL=+45°, FR=-45°, RL=-45°, RR=+45°
    mecanum_angles = [np.pi / 4, -np.pi / 4, -np.pi / 4, np.pi / 4]  # radians

    carb.log_info("Configuring mecanum wheels...")
    for joint_name, angle in zip(wheel_dof_names, mecanum_angles):
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

            # INCREASED DAMPING to reduce jitter from wheel fighting
            drive_api = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
            drive_api.GetDampingAttr().Set(10.0)  # Increased from 1.0
            drive_api.GetStiffnessAttr().Set(0.0)
            carb.log_info(f"  ✓ Configured wheel: {joint_name}")
        else:
            carb.log_warn(f"  ✗ Joint not found: {joint_name}")

    # Create test scene with target cube
    carb.log_info("Creating test scene with target cube...")

    # Target cube (Yellow - this is what the robot should find)
    target_cube = stage.DefinePrim("/World/Obstacles/TargetCube", "Cube")
    UsdGeom.Xform(target_cube).AddTranslateOp().Set(Gf.Vec3f(2.0, 1.5, 0.25))
    UsdGeom.Xform(target_cube).AddScaleOp().Set(Gf.Vec3f(0.5, 0.5, 0.5))
    target_cube.GetAttribute("primvars:displayColor").Set([(1.0, 1.0, 0.0)])  # Yellow
    UsdPhysics.CollisionAPI.Apply(target_cube)
    UsdPhysics.RigidBodyAPI.Apply(target_cube)

    # Box obstacle 1 (Red)
    box1_prim = stage.DefinePrim("/World/Obstacles/Box1", "Cube")
    UsdGeom.Xform(box1_prim).AddTranslateOp().Set(Gf.Vec3f(1.5, -0.8, 0.25))
    UsdGeom.Xform(box1_prim).AddScaleOp().Set(Gf.Vec3f(0.4, 0.4, 0.5))
    box1_prim.GetAttribute("primvars:displayColor").Set([(0.9, 0.3, 0.3)])
    UsdPhysics.CollisionAPI.Apply(box1_prim)
    UsdPhysics.RigidBodyAPI.Apply(box1_prim)

    # Box obstacle 2 (Green)
    box2_prim = stage.DefinePrim("/World/Obstacles/Box2", "Cube")
    UsdGeom.Xform(box2_prim).AddTranslateOp().Set(Gf.Vec3f(-1.2, -1.2, 0.2))
    UsdGeom.Xform(box2_prim).AddScaleOp().Set(Gf.Vec3f(0.3, 0.3, 0.4))
    box2_prim.GetAttribute("primvars:displayColor").Set([(0.3, 0.9, 0.3)])
    UsdPhysics.CollisionAPI.Apply(box2_prim)
    UsdPhysics.RigidBodyAPI.Apply(box2_prim)

    # Cylinder obstacle (Blue)
    cylinder_prim = stage.DefinePrim("/World/Obstacles/Cylinder1", "Cylinder")
    UsdGeom.Xform(cylinder_prim).AddTranslateOp().Set(Gf.Vec3f(-1.8, 1.0, 0.3))
    UsdGeom.Xform(cylinder_prim).AddScaleOp().Set(Gf.Vec3f(0.15, 0.15, 0.3))
    cylinder_prim.GetAttribute("primvars:displayColor").Set([(0.3, 0.3, 0.9)])
    UsdPhysics.CollisionAPI.Apply(cylinder_prim)
    UsdPhysics.RigidBodyAPI.Apply(cylinder_prim)

    # Wall (Gray)
    wall_prim = stage.DefinePrim("/World/Obstacles/Wall1", "Cube")
    UsdGeom.Xform(wall_prim).AddTranslateOp().Set(Gf.Vec3f(0.0, 2.5, 0.4))
    UsdGeom.Xform(wall_prim).AddScaleOp().Set(Gf.Vec3f(3.0, 0.1, 0.8))
    wall_prim.GetAttribute("primvars:displayColor").Set([(0.6, 0.6, 0.6)])
    UsdPhysics.CollisionAPI.Apply(wall_prim)
    UsdPhysics.RigidBodyAPI.Apply(wall_prim)

    # Visual reference markers
    forward_marker = stage.DefinePrim("/World/Markers/ForwardMarker", "Cube")
    UsdGeom.Xform(forward_marker).AddTranslateOp().Set(Gf.Vec3f(1.0, 0.0, 0.05))
    UsdGeom.Xform(forward_marker).AddScaleOp().Set(Gf.Vec3f(0.3, 0.1, 0.1))
    forward_marker.GetAttribute("primvars:displayColor").Set([(1.0, 0.0, 0.0)])  # Red = Forward

    left_marker = stage.DefinePrim("/World/Markers/LeftMarker", "Cube")
    UsdGeom.Xform(left_marker).AddTranslateOp().Set(Gf.Vec3f(0.0, 1.0, 0.05))
    UsdGeom.Xform(left_marker).AddScaleOp().Set(Gf.Vec3f(0.1, 0.3, 0.1))
    left_marker.GetAttribute("primvars:displayColor").Set([(0.0, 1.0, 0.0)])  # Green = Left

    carb.log_info("  ✓ Created test scene")

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

    # DEBUG: Print what the controller extracted
    carb.log_info("=" * 60)
    carb.log_info("CONTROLLER PARAMETERS:")
    carb.log_info(f"Wheel radius: {wheel_radius_params}")
    carb.log_info(f"Mecanum angles: {[np.degrees(a) for a in mecanum_angles_params]}°")
    carb.log_info(f"Wheel axis: {wheel_axis}")
    carb.log_info(f"Up axis: {up_axis}")
    carb.log_info("=" * 60)

    my_controller = HolonomicController(
        name="holonomic_controller",
        wheel_radius=wheel_radius_params,
        wheel_positions=wheel_positions,
        wheel_orientations=wheel_orientations,
        mecanum_angles=mecanum_angles_params,
        wheel_axis=wheel_axis,
        up_axis=up_axis,
    )

    carb.log_info("  ✓ HolonomicController initialized")

    # Initialize keyboard controller
    keyboard_ctrl = KeyboardController()

    # Reset world
    my_world.reset()

    carb.log_info("=" * 80)
    carb.log_info("LUKEBOT TELEOPERATION READY!")
    carb.log_info("=" * 80)
    carb.log_info(f"Robot location: {robot_prim_path}")
    carb.log_info("")
    carb.log_info("FIXES APPLIED:")
    carb.log_info("  ✓ Axis swap compensation (lateral ↔ forward)")
    carb.log_info("  ✓ Increased damping to reduce jitter (10.0)")
    carb.log_info("  ✓ Rotation direction correction")
    carb.log_info("")
    carb.log_info("KEYBOARD CONTROLS:")
    carb.log_info("  W - Move forward (toward RED marker)")
    carb.log_info("  S - Move backward")
    carb.log_info("  A - Strafe left (toward GREEN marker)")
    carb.log_info("  D - Strafe right")
    carb.log_info("  Q - Rotate counter-clockwise")
    carb.log_info("  E - Rotate clockwise")
    carb.log_info("  SPACE - Stop all movement")
    carb.log_info("")
    carb.log_info("OBJECTIVE:")
    carb.log_info("  Navigate to the YELLOW cube using keyboard controls")
    carb.log_info("=" * 80)

    step_count = 0
    reset_needed = False
    last_command = [0.0, 0.0, 0.0]
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

                # Get keyboard command and apply to robot
                command = keyboard_ctrl.get_command()

                # FIX: Swap lateral and forward to compensate for URDF axis orientation
                # Original command: [forward, lateral, rotation]
                # Fixed command: [lateral, forward, -rotation]
                command_fixed = [command[1], command[0], -command[2]]

                # Only apply command if robot exists and is valid
                if my_lukebot:
                    wheel_actions = my_controller.forward(command=command_fixed)
                    my_lukebot.apply_wheel_actions(wheel_actions)

                    # Periodic diagnostics
                    if step_count % 60 == 0 and any(c != 0 for c in command):
                        current_position, _ = my_lukebot.get_world_pose()
                        if last_position is not None:
                            delta_pos = current_position - last_position
                            delta_magnitude = np.linalg.norm(delta_pos[:2])
                            if delta_magnitude > 0.001:
                                direction = np.arctan2(delta_pos[1], delta_pos[0])
                                direction_deg = np.degrees(direction)

                                carb.log_info(f"[{step_count}] Pos: ({current_position[0]:+.2f}, {current_position[1]:+.2f}) | "
                                            f"Move: {direction_deg:+.0f}° | Cmd: F={command[0]:.1f} L={command[1]:.1f} R={command[2]:.1f}")
                        last_position = current_position.copy()

                # Log command changes
                if command != last_command and any(c != 0 for c in command):
                    carb.log_info(f">> Command: Forward={command[0]:.2f}, Lateral={command[1]:.2f}, Rotation={command[2]:.2f}")
                    last_command = command.copy()

                step_count += 1

    finally:
        # Clean up keyboard controller
        keyboard_ctrl.shutdown()
        simulation_app.close()


if __name__ == "__main__":
    main()
