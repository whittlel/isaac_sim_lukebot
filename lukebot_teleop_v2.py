# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Lukebot Keyboard Teleoperation V2
Uses the new motion controller abstraction for sim-to-real transfer

Controls:
  W - Move forward
  S - Move backward
  A - Strafe left
  D - Strafe right
  Q - Rotate counter-clockwise
  E - Rotate clockwise
  SPACE - Stop
  ESC - Exit
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import carb
import numpy as np
import omni.appwindow
import omni.kit.commands
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import get_current_stage
from isaacsim.core.utils.viewports import set_camera_view
from pxr import Gf, UsdGeom, UsdPhysics

# Import our motion controller
from lukebot_motion_controller import create_motion_controller

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
        return (self.forward_speed, self.lateral_speed, self.rotation_speed)

    def shutdown(self):
        """Clean up keyboard subscription"""
        if self._sub_keyboard:
            self._input.unsubscribe_to_keyboard_events(self._keyboard, self._sub_keyboard)
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
    carb.log_info("LUKEBOT KEYBOARD TELEOPERATION V2")
    carb.log_info("Using Manual Mecanum Kinematics (Sim-to-Real Ready)")
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

    # Configure wheel drive properties
    wheel_dof_names = [
        "front_left_wheel_joint",
        "front_right_wheel_joint",
        "rear_left_wheel_joint",
        "rear_right_wheel_joint",
    ]

    carb.log_info("Configuring wheel drive properties...")
    for joint_name in wheel_dof_names:
        joint_path = f"{robot_prim_path}/joints/{joint_name}"
        joint_prim = stage.GetPrimAtPath(joint_path)

        if joint_prim and joint_prim.IsValid():
            drive_api = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
            drive_api.GetDampingAttr().Set(100.0)
            drive_api.GetStiffnessAttr().Set(0.0)
            carb.log_info(f"  ✓ Configured {joint_name}")
        else:
            carb.log_warn(f"  ✗ Joint not found: {joint_name}")

    # Create test scene with target cube
    carb.log_info("Creating test scene...")

    # Target cube (Yellow)
    target_cube = stage.DefinePrim("/World/Obstacles/TargetCube", "Cube")
    UsdGeom.Xform(target_cube).AddTranslateOp().Set(Gf.Vec3f(2.0, 1.5, 0.25))
    UsdGeom.Xform(target_cube).AddScaleOp().Set(Gf.Vec3f(0.5, 0.5, 0.5))
    target_cube.GetAttribute("primvars:displayColor").Set([(1.0, 1.0, 0.0)])
    UsdPhysics.CollisionAPI.Apply(target_cube)
    UsdPhysics.RigidBodyAPI.Apply(target_cube)

    # Box obstacle 1 (Red)
    box1_prim = stage.DefinePrim("/World/Obstacles/Box1", "Cube")
    UsdGeom.Xform(box1_prim).AddTranslateOp().Set(Gf.Vec3f(1.5, -0.8, 0.25))
    UsdGeom.Xform(box1_prim).AddScaleOp().Set(Gf.Vec3f(0.4, 0.4, 0.5))
    box1_prim.GetAttribute("primvars:displayColor").Set([(0.9, 0.3, 0.3)])
    UsdPhysics.CollisionAPI.Apply(box1_prim)
    UsdPhysics.RigidBodyAPI.Apply(box1_prim)

    # Cylinder obstacle (Blue)
    cylinder_prim = stage.DefinePrim("/World/Obstacles/Cylinder1", "Cylinder")
    UsdGeom.Xform(cylinder_prim).AddTranslateOp().Set(Gf.Vec3f(-1.8, 1.0, 0.3))
    UsdGeom.Xform(cylinder_prim).AddScaleOp().Set(Gf.Vec3f(0.15, 0.15, 0.3))
    cylinder_prim.GetAttribute("primvars:displayColor").Set([(0.3, 0.3, 0.9)])
    UsdPhysics.CollisionAPI.Apply(cylinder_prim)
    UsdPhysics.RigidBodyAPI.Apply(cylinder_prim)

    carb.log_info("  ✓ Created test scene")

    # Add Lukebot as a Robot (NOT WheeledRobot)
    my_lukebot = my_world.scene.add(
        Robot(
            prim_path=robot_prim_path,
            name="my_lukebot",
            position=np.array([0, 0.0, 0.1]),
        )
    )

    # Create motion controller with actual Lukebot dimensions
    carb.log_info("Creating motion controller...")
    motion_controller = create_motion_controller(
        robot=my_lukebot,
        use_simulation=True,
        wheel_base=0.30,      # 300mm between front/rear axles
        track_width=0.34,     # 340mm between left/right wheels
        wheel_radius=0.05     # 50mm wheel radius
    )
    carb.log_info("  ✓ Motion controller initialized with manual mecanum kinematics")

    # Initialize keyboard controller
    keyboard_ctrl = KeyboardController()

    # Reset world
    my_world.reset()

    carb.log_info("\n" + "=" * 80)
    carb.log_info("LUKEBOT TELEOPERATION V2 READY!")
    carb.log_info("=" * 80)
    carb.log_info(f"Robot location: {robot_prim_path}")
    carb.log_info("")
    carb.log_info("KEYBOARD CONTROLS:")
    carb.log_info("  W - Move forward")
    carb.log_info("  S - Move backward")
    carb.log_info("  A - Strafe left")
    carb.log_info("  D - Strafe right")
    carb.log_info("  Q - Rotate counter-clockwise")
    carb.log_info("  E - Rotate clockwise")
    carb.log_info("  SPACE - Stop all movement")
    carb.log_info("")
    carb.log_info("OBJECTIVE:")
    carb.log_info("  Navigate to the YELLOW cube using keyboard controls")
    carb.log_info("  Test omnidirectional movement (forward, strafe, rotate)")
    carb.log_info("=" * 80 + "\n")

    step_count = 0
    reset_needed = False
    last_command = (0.0, 0.0, 0.0)

    try:
        while simulation_app.is_running():
            my_world.step(render=True)

            if my_world.is_stopped() and not reset_needed:
                reset_needed = True

            if my_world.is_playing():
                if reset_needed:
                    my_world.reset()
                    motion_controller.reset()
                    reset_needed = False
                    step_count = 0

                # Get keyboard command
                vx, vy, omega = keyboard_ctrl.get_command()

                # Apply motion command using the abstracted interface
                # This is the SAME code you'll use on the Jetson!
                motion_controller.set_velocity(vx, vy, omega)

                # Update robot pose (only needed in simulation with physics bypass)
                # On real robot, this does nothing (motion is handled by hardware)
                motion_controller.update(dt=1.0/60.0)  # Assuming 60 FPS

                # Log command changes
                current_command = (vx, vy, omega)
                if current_command != last_command and any(c != 0 for c in current_command):
                    carb.log_info(f"Motion: vx={vx:.2f} m/s, vy={vy:.2f} m/s, omega={omega:.2f} rad/s")

                    # Optionally log actual velocity
                    if step_count % 30 == 0:
                        actual_vx, actual_vy, actual_omega = motion_controller.get_actual_velocity()
                        carb.log_info(f"  Actual: vx={actual_vx:.2f}, vy={actual_vy:.2f}, omega={actual_omega:.2f}")

                    last_command = current_command

                step_count += 1

    finally:
        # Clean up
        keyboard_ctrl.shutdown()
        simulation_app.close()


if __name__ == "__main__":
    main()
