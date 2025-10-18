# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Lukebot Mecanum Wheel Angle Configuration Tester
Test different mecanum angle configurations to find the correct one

Controls:
  W - Move forward ONLY (simple test)
  S - Move backward ONLY
  A - Strafe left ONLY
  D - Strafe right ONLY

  1-5 - Switch between angle configurations
  SPACE - Stop
  ESC - Exit

This will help us find the correct mecanum angle configuration.
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

# Different mecanum angle configurations to test
# Format: [FL, FR, RL, RR] in degrees
ANGLE_CONFIGS = {
    1: {
        "name": "Standard X-Pattern",
        "angles": [45, -45, -45, 45],
        "description": "FL=+45°, FR=-45°, RL=-45°, RR=+45° (most common)"
    },
    2: {
        "name": "Inverted X-Pattern",
        "angles": [-45, 45, 45, -45],
        "description": "FL=-45°, FR=+45°, RL=+45°, RR=-45° (opposite rollers)"
    },
    3: {
        "name": "Mirrored Front/Back",
        "angles": [45, -45, 45, -45],
        "description": "FL=+45°, FR=-45°, RL=+45°, RR=-45° (diagonal pairs)"
    },
    4: {
        "name": "O-Pattern",
        "angles": [-45, -45, 45, 45],
        "description": "FL=-45°, FR=-45°, RL=+45°, RR=+45° (O-shaped)"
    },
    5: {
        "name": "All Positive",
        "angles": [45, 45, 45, 45],
        "description": "All wheels +45° (testing)"
    }
}


class KeyboardController:
    """Simple keyboard controller with config switching"""

    def __init__(self):
        self.forward_speed = 0.0
        self.lateral_speed = 0.0
        self.rotation_speed = 0.0
        self.max_linear_speed = 0.3  # m/s (slower for testing)
        self.max_angular_speed = 0.5  # rad/s
        self.current_config = 1
        self.config_changed = False

        # Get keyboard interface
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()
        self._sub_keyboard = self._input.subscribe_to_keyboard_events(self._keyboard, self._on_keyboard_event)

        carb.log_info("=" * 80)
        carb.log_info("MECANUM ANGLE CONFIGURATION TESTER")
        carb.log_info("=" * 80)
        carb.log_info("Controls:")
        carb.log_info("  W/S - Forward/Backward ONLY")
        carb.log_info("  A/D - Strafe Left/Right ONLY")
        carb.log_info("  SPACE - Stop")
        carb.log_info("")
        carb.log_info("Configuration Switch:")
        for key, config in ANGLE_CONFIGS.items():
            carb.log_info(f"  {key} - {config['name']}: {config['description']}")
        carb.log_info("=" * 80)

    def _on_keyboard_event(self, event, *args, **kwargs):
        """Handle keyboard events"""
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            # Movement keys
            if event.input == carb.input.KeyboardInput.W:
                self.forward_speed = self.max_linear_speed
                self.lateral_speed = 0.0
                self.rotation_speed = 0.0
            elif event.input == carb.input.KeyboardInput.S:
                self.forward_speed = -self.max_linear_speed
                self.lateral_speed = 0.0
                self.rotation_speed = 0.0
            elif event.input == carb.input.KeyboardInput.A:
                self.lateral_speed = self.max_linear_speed
                self.forward_speed = 0.0
                self.rotation_speed = 0.0
            elif event.input == carb.input.KeyboardInput.D:
                self.lateral_speed = -self.max_linear_speed
                self.forward_speed = 0.0
                self.rotation_speed = 0.0
            elif event.input == carb.input.KeyboardInput.SPACE:
                self.forward_speed = 0.0
                self.lateral_speed = 0.0
                self.rotation_speed = 0.0

            # Config selection (1-5)
            elif event.input == carb.input.KeyboardInput.KEY_1:
                self.current_config = 1
                self.config_changed = True
            elif event.input == carb.input.KeyboardInput.KEY_2:
                self.current_config = 2
                self.config_changed = True
            elif event.input == carb.input.KeyboardInput.KEY_3:
                self.current_config = 3
                self.config_changed = True
            elif event.input == carb.input.KeyboardInput.KEY_4:
                self.current_config = 4
                self.config_changed = True
            elif event.input == carb.input.KeyboardInput.KEY_5:
                self.current_config = 5
                self.config_changed = True

        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            # Stop movement when key is released
            if event.input in [carb.input.KeyboardInput.W, carb.input.KeyboardInput.S]:
                self.forward_speed = 0.0
            elif event.input in [carb.input.KeyboardInput.A, carb.input.KeyboardInput.D]:
                self.lateral_speed = 0.0

        return True

    def get_command(self):
        """Get current movement command [forward, lateral, rotation]"""
        return [self.forward_speed, self.lateral_speed, self.rotation_speed]

    def shutdown(self):
        """Clean up keyboard subscription"""
        if self._sub_keyboard:
            self._input.unsubscribe_from_keyboard_events(self._keyboard, self._sub_keyboard)
            self._sub_keyboard = None


def apply_mecanum_config(stage, robot_prim_path, wheel_dof_names, config_num):
    """Apply a specific mecanum angle configuration to the robot"""

    config = ANGLE_CONFIGS[config_num]
    wheel_radius = 0.050  # 50mm

    carb.log_info("=" * 80)
    carb.log_info(f"APPLYING CONFIG {config_num}: {config['name']}")
    carb.log_info(f"Description: {config['description']}")
    carb.log_info("=" * 80)

    # Convert degrees to radians
    mecanum_angles_rad = [np.radians(angle) for angle in config['angles']]

    for i, (joint_name, angle_deg, angle_rad) in enumerate(zip(wheel_dof_names, config['angles'], mecanum_angles_rad)):
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
            # Set mecanum wheel attributes
            if not joint_prim.HasAttribute("isaacmecanumwheel:radius"):
                joint_prim.CreateAttribute("isaacmecanumwheel:radius", Sdf.ValueTypeNames.Float).Set(wheel_radius)
            else:
                joint_prim.GetAttribute("isaacmecanumwheel:radius").Set(wheel_radius)

            if not joint_prim.HasAttribute("isaacmecanumwheel:angle"):
                joint_prim.CreateAttribute("isaacmecanumwheel:angle", Sdf.ValueTypeNames.Float).Set(angle_rad)
            else:
                joint_prim.GetAttribute("isaacmecanumwheel:angle").Set(angle_rad)

            # Higher damping to prevent instability
            drive_api = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
            drive_api.GetDampingAttr().Set(50.0)  # Very high damping to prevent jumping
            drive_api.GetStiffnessAttr().Set(0.0)

            carb.log_info(f"  [{i}] {joint_name}: {angle_deg:+4.0f}° ({angle_rad:+.3f} rad)")
        else:
            carb.log_warn(f"  ✗ Joint not found: {joint_name}")

    carb.log_info("=" * 80)
    return mecanum_angles_rad


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

    # Apply initial configuration (Config 1)
    apply_mecanum_config(stage, robot_prim_path, wheel_dof_names, 1)

    # Visual reference markers
    forward_marker = stage.DefinePrim("/World/Markers/ForwardMarker", "Cube")
    UsdGeom.Xform(forward_marker).AddTranslateOp().Set(Gf.Vec3f(1.5, 0.0, 0.05))
    UsdGeom.Xform(forward_marker).AddScaleOp().Set(Gf.Vec3f(0.4, 0.1, 0.1))
    forward_marker.GetAttribute("primvars:displayColor").Set([(1.0, 0.0, 0.0)])  # Red = Forward

    left_marker = stage.DefinePrim("/World/Markers/LeftMarker", "Cube")
    UsdGeom.Xform(left_marker).AddTranslateOp().Set(Gf.Vec3f(0.0, 1.5, 0.05))
    UsdGeom.Xform(left_marker).AddScaleOp().Set(Gf.Vec3f(0.1, 0.4, 0.1))
    left_marker.GetAttribute("primvars:displayColor").Set([(0.0, 1.0, 0.0)])  # Green = Left

    # Add Lukebot as a WheeledRobot to the scene
    my_lukebot = my_world.scene.add(
        WheeledRobot(
            prim_path=robot_prim_path,
            name="my_lukebot",
            wheel_dof_names=wheel_dof_names,
            create_robot=False,
            position=np.array([0, 0.0, 0.1]),
        )
    )

    # Initialize keyboard controller
    keyboard_ctrl = KeyboardController()

    # These will be recreated when config changes
    my_controller = None
    controller_needs_init = True

    # Reset world
    my_world.reset()

    carb.log_info("=" * 80)
    carb.log_info("ANGLE CONFIGURATION TESTER READY!")
    carb.log_info("=" * 80)
    carb.log_info("Starting with Config 1 (Standard X-Pattern)")
    carb.log_info("")
    carb.log_info("TEST PROCEDURE:")
    carb.log_info("1. Press W - should move toward RED marker (forward)")
    carb.log_info("2. Press A - should move toward GREEN marker (left)")
    carb.log_info("3. If wrong behavior, press 2/3/4/5 to try other configs")
    carb.log_info("4. Find which config works correctly")
    carb.log_info("=" * 80)

    step_count = 0
    reset_needed = False
    last_position = None
    current_config = 1

    try:
        while simulation_app.is_running():
            my_world.step(render=True)

            if my_world.is_stopped() and not reset_needed:
                reset_needed = True

            if my_world.is_playing():
                if reset_needed:
                    my_world.reset()
                    if my_controller:
                        my_controller.reset()
                    reset_needed = False
                    step_count = 0
                    last_position = None
                    controller_needs_init = True

                # Check if configuration changed
                if keyboard_ctrl.config_changed or controller_needs_init:
                    if keyboard_ctrl.config_changed:
                        current_config = keyboard_ctrl.current_config
                        apply_mecanum_config(stage, robot_prim_path, wheel_dof_names, current_config)
                        keyboard_ctrl.config_changed = False

                        config = ANGLE_CONFIGS[current_config]
                        carb.log_info("")
                        carb.log_info("=" * 80)
                        carb.log_info(f"SWITCHED TO CONFIG {current_config}: {config['name']}")
                        carb.log_info(f"{config['description']}")
                        carb.log_info("Try W (forward) and A (left) now!")
                        carb.log_info("=" * 80)
                        carb.log_info("")

                    # Recreate controller with new parameters
                    lukebot_setup = HolonomicRobotUsdSetup(
                        robot_prim_path=robot_prim_path,
                        com_prim_path=f"{robot_prim_path}/chassis_link"
                    )

                    (
                        wheel_radius_params,
                        wheel_positions,
                        wheel_orientations,
                        mecanum_angles_params,
                        wheel_axis,
                        up_axis,
                    ) = lukebot_setup.get_holonomic_controller_params()

                    my_controller = HolonomicController(
                        name="holonomic_controller",
                        wheel_radius=wheel_radius_params,
                        wheel_positions=wheel_positions,
                        wheel_orientations=wheel_orientations,
                        mecanum_angles=mecanum_angles_params,
                        wheel_axis=wheel_axis,
                        up_axis=up_axis,
                    )
                    controller_needs_init = False

                # Get keyboard command
                command = keyboard_ctrl.get_command()

                # Apply command with axis swap (based on earlier diagnosis)
                command_fixed = [command[1], command[0], -command[2]]

                if my_lukebot and my_controller:
                    wheel_actions = my_controller.forward(command=command_fixed)
                    my_lukebot.apply_wheel_actions(wheel_actions)

                    # Diagnostics every 60 steps
                    if step_count % 60 == 0 and step_count > 0 and any(c != 0 for c in command):
                        current_position, _ = my_lukebot.get_world_pose()
                        if last_position is not None:
                            delta_pos = current_position - last_position
                            delta_magnitude = np.linalg.norm(delta_pos[:2])
                            if delta_magnitude > 0.01:
                                direction = np.arctan2(delta_pos[1], delta_pos[0])
                                direction_deg = np.degrees(direction)

                                status = ""
                                if command[0] > 0:  # Forward command
                                    if abs(direction_deg) < 20:
                                        status = "✓ CORRECT"
                                    else:
                                        status = f"✗ OFF BY {abs(direction_deg):.0f}°"
                                elif command[1] > 0:  # Left command
                                    if abs(direction_deg - 90) < 20:
                                        status = "✓ CORRECT"
                                    else:
                                        status = f"✗ OFF BY {abs(direction_deg - 90):.0f}°"

                                carb.log_info(f"[Config {current_config}] Direction: {direction_deg:+.0f}° | "
                                            f"Cmd: F={command[0]:.1f} L={command[1]:.1f} | {status}")

                        last_position = current_position.copy()

                step_count += 1

    finally:
        keyboard_ctrl.shutdown()
        simulation_app.close()


if __name__ == "__main__":
    main()
