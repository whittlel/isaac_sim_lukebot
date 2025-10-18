# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Lukebot Wheel Configuration Diagnostic Test
Tests each wheel individually and each movement direction to diagnose the issue
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import carb
import numpy as np
import omni.kit.commands
from isaacsim.core.api import World
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import get_current_stage
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.robot.wheeled_robots.controllers.holonomic_controller import HolonomicController
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from isaacsim.robot.wheeled_robots.robots.holonomic_robot_usd_setup import HolonomicRobotUsdSetup
from pxr import Gf, Sdf, UsdGeom, UsdPhysics

enable_extension("omni.isaac.sensor")


def main():
    # Create world
    my_world = World(stage_units_in_meters=1.0)
    my_world.scene.add_default_ground_plane()

    # Set camera view
    set_camera_view(eye=[4.0, 4.0, 3.0], target=[0.0, 0.0, 0.3], camera_prim_path="/OmniverseKit_Persp")

    # Get URDF path
    urdf_path = "C:/Users/Luke/Desktop/issac_sim/source/extensions/isaacsim.asset.importer.urdf/data/urdf/robots/lukebot/urdf/lukebot.urdf"

    carb.log_info("=" * 80)
    carb.log_info("LUKEBOT WHEEL CONFIGURATION DIAGNOSTIC TEST")
    carb.log_info("=" * 80)

    # Import URDF
    status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    import_config.merge_fixed_joints = False
    import_config.convex_decomp = False
    import_config.import_inertia_tensor = True
    import_config.fix_base = False
    import_config.make_default_prim = False
    import_config.self_collision = False
    import_config.create_physics_scene = False
    import_config.distance_scale = 1.0

    omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=urdf_path,
        import_config=import_config,
    )

    stage = get_current_stage()

    # Find robot
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

    wheel_dof_names = [
        "front_left_wheel_joint",
        "front_right_wheel_joint",
        "rear_left_wheel_joint",
        "rear_right_wheel_joint",
    ]

    # Test multiple configurations
    test_configs = [
        {
            "name": "Original (Kaya-style)",
            "angles": [np.pi / 4, -np.pi / 4, -np.pi / 4, np.pi / 4],
            "swap": False,
        },
        {
            "name": "Negated angles",
            "angles": [-np.pi / 4, np.pi / 4, np.pi / 4, -np.pi / 4],
            "swap": False,
        },
        {
            "name": "Original + axis swap",
            "angles": [np.pi / 4, -np.pi / 4, -np.pi / 4, np.pi / 4],
            "swap": True,
        },
        {
            "name": "All positive 45°",
            "angles": [np.pi / 4, np.pi / 4, np.pi / 4, np.pi / 4],
            "swap": False,
        },
    ]

    current_config_index = 0
    wheel_radius = 0.050

    # Configure initial wheel attributes
    def configure_wheels(mecanum_angles):
        carb.log_info(f"\nConfiguring wheels with angles: {[np.degrees(a) for a in mecanum_angles]}")
        for joint_name, angle in zip(wheel_dof_names, mecanum_angles):
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
                carb.log_info(f"  ✓ {joint_name}: {np.degrees(angle):.1f}°")

    # Initial configuration
    configure_wheels(test_configs[current_config_index]["angles"])

    # Add robot to scene
    my_lukebot = my_world.scene.add(
        WheeledRobot(
            prim_path=robot_prim_path,
            name="my_lukebot",
            wheel_dof_names=wheel_dof_names,
            create_robot=False,
            position=np.array([0, 0.0, 0.1]),
        )
    )

    # Setup HolonomicController
    def setup_controller():
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

        carb.log_warn("=" * 60)
        carb.log_warn("EXTRACTED CONTROLLER PARAMETERS:")
        carb.log_warn(f"Wheel radius: {wheel_radius_params}")
        carb.log_warn(f"Wheel positions:\n{wheel_positions}")
        carb.log_warn(f"Mecanum angles (degrees): {[np.degrees(a) for a in mecanum_angles_params]}")
        carb.log_warn(f"Wheel axis: {wheel_axis}")
        carb.log_warn(f"Up axis: {up_axis}")
        carb.log_warn("=" * 60)

        controller = HolonomicController(
            name="holonomic_controller",
            wheel_radius=wheel_radius_params,
            wheel_positions=wheel_positions,
            wheel_orientations=wheel_orientations,
            mecanum_angles=mecanum_angles_params,
            wheel_axis=wheel_axis,
            up_axis=up_axis,
        )
        return controller

    my_controller = setup_controller()
    my_world.reset()

    # Test sequence
    test_sequence = [
        {"name": "Forward (X+)", "command": [0.3, 0.0, 0.0], "duration": 200},
        {"name": "Stop", "command": [0.0, 0.0, 0.0], "duration": 50},
        {"name": "Backward (X-)", "command": [-0.3, 0.0, 0.0], "duration": 200},
        {"name": "Stop", "command": [0.0, 0.0, 0.0], "duration": 50},
        {"name": "Strafe Left (Y+)", "command": [0.0, 0.3, 0.0], "duration": 200},
        {"name": "Stop", "command": [0.0, 0.0, 0.0], "duration": 50},
        {"name": "Strafe Right (Y-)", "command": [0.0, -0.3, 0.0], "duration": 200},
        {"name": "Stop", "command": [0.0, 0.0, 0.0], "duration": 50},
        {"name": "Rotate CCW", "command": [0.0, 0.0, 0.5], "duration": 200},
        {"name": "Stop", "command": [0.0, 0.0, 0.0], "duration": 50},
        {"name": "Rotate CW", "command": [0.0, 0.0, -0.5], "duration": 200},
        {"name": "Stop", "command": [0.0, 0.0, 0.0], "duration": 100},
    ]

    carb.log_info("\n" + "=" * 80)
    carb.log_info(f"TESTING CONFIGURATION: {test_configs[current_config_index]['name']}")
    carb.log_info("=" * 80)
    carb.log_info("\nTest Sequence:")
    for i, test in enumerate(test_sequence):
        carb.log_info(f"  {i+1}. {test['name']}: command={test['command']}, duration={test['duration']} frames")

    carb.log_info("\n" + "=" * 80)
    carb.log_info("WATCH THE ROBOT'S MOVEMENT AND NOTE:")
    carb.log_info("  - Does forward command move forward?")
    carb.log_info("  - Does strafe command move sideways?")
    carb.log_info("  - Does rotate command spin in place?")
    carb.log_info("=" * 80 + "\n")

    step_count = 0
    test_index = 0
    test_start_step = 0
    reset_needed = False

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
                    test_index = 0
                    test_start_step = 0

                # Execute test sequence
                if test_index < len(test_sequence):
                    current_test = test_sequence[test_index]
                    elapsed = step_count - test_start_step

                    if elapsed == 0:
                        carb.log_info(f"\n>>> TEST {test_index + 1}/{len(test_sequence)}: {current_test['name']}")
                        carb.log_info(f"    Command: {current_test['command']}")

                    command = current_test["command"]

                    # Apply axis swap if configured
                    if test_configs[current_config_index]["swap"]:
                        command = [command[1], command[0], -command[2]]

                    wheel_actions = my_controller.forward(command=command)
                    my_lukebot.apply_wheel_actions(wheel_actions)

                    # Print wheel velocities for first frame of each test
                    if elapsed == 0 and any(c != 0 for c in command):
                        carb.log_info(f"    Computed wheel velocities: {wheel_actions.joint_velocities}")

                    if elapsed >= current_test["duration"]:
                        test_index += 1
                        test_start_step = step_count + 1

                else:
                    # All tests complete - hold position
                    my_lukebot.apply_wheel_actions(my_controller.forward(command=[0.0, 0.0, 0.0]))

                    if step_count % 60 == 0:
                        carb.log_info("\n" + "=" * 80)
                        carb.log_info("TEST SEQUENCE COMPLETE!")
                        carb.log_info(f"Configuration tested: {test_configs[current_config_index]['name']}")
                        carb.log_info("=" * 80)
                        carb.log_info("\nDid the movements match expectations?")
                        carb.log_info("Press STOP to end test, or wait for auto-restart...")
                        carb.log_info("=" * 80 + "\n")

                step_count += 1

    finally:
        simulation_app.close()


if __name__ == "__main__":
    main()
