# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Lukebot Automatic Wheel Velocity Test
Automatically tests each movement direction and prints wheel velocities

This will help diagnose why strafe causes rotation
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
from pxr import Gf, Sdf, UsdGeom, UsdPhysics

# Enable necessary extensions
enable_extension("omni.isaac.sensor")


def main():
    # Create world
    my_world = World(stage_units_in_meters=1.0)
    my_world.scene.add_default_ground_plane()

    # Set camera view
    set_camera_view(eye=[3.0, 3.0, 2.5], target=[0.0, 0.0, 0.3], camera_prim_path="/OmniverseKit_Persp")

    # Get URDF path
    urdf_path = "C:/Users/Luke/Desktop/issac_sim/source/extensions/isaacsim.asset.importer.urdf/data/urdf/robots/lukebot/urdf/lukebot.urdf"

    carb.log_info("=" * 80)
    carb.log_info("LUKEBOT AUTOMATIC WHEEL VELOCITY TEST")
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

    # MANUAL CONTROLLER PARAMETERS
    wheel_radius = np.array([0.050, 0.050, 0.050, 0.050])

    wheel_positions = np.array([
        [0.15, 0.17, -0.05],   # Front Left
        [0.15, -0.17, -0.05],  # Front Right
        [-0.15, 0.17, -0.05],  # Rear Left
        [-0.15, -0.17, -0.05], # Rear Right
    ])

    wheel_orientations = np.array([
        [1.0, 0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0, 0.0],
    ])

    mecanum_angles = np.array([
        np.pi / 4,   # Front Left: +45°
        -np.pi / 4,  # Front Right: -45°
        -np.pi / 4,  # Rear Left: -45°
        np.pi / 4,   # Rear Right: +45°
    ])

    wheel_axis = np.array([0.0, 1.0, 0.0])
    up_axis = np.array([0.0, 0.0, 1.0])

    # Configure mecanum wheels in USD
    carb.log_info("Configuring mecanum wheels...")
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
            if not joint_prim.HasAttribute("isaacmecanumwheel:radius"):
                joint_prim.CreateAttribute("isaacmecanumwheel:radius", Sdf.ValueTypeNames.Float).Set(wheel_radius[i])
            else:
                joint_prim.GetAttribute("isaacmecanumwheel:radius").Set(wheel_radius[i])

            if not joint_prim.HasAttribute("isaacmecanumwheel:angle"):
                joint_prim.CreateAttribute("isaacmecanumwheel:angle", Sdf.ValueTypeNames.Float).Set(angle)
            else:
                joint_prim.GetAttribute("isaacmecanumwheel:angle").Set(angle)

            drive_api = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
            drive_api.GetDampingAttr().Set(20.0)
            drive_api.GetStiffnessAttr().Set(0.0)

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

    # Create HolonomicController
    carb.log_info("Creating HolonomicController...")
    my_controller = HolonomicController(
        name="holonomic_controller",
        wheel_radius=wheel_radius,
        wheel_positions=wheel_positions,
        wheel_orientations=wheel_orientations,
        mecanum_angles=mecanum_angles,
        wheel_axis=wheel_axis,
        up_axis=up_axis,
    )

    # Reset world
    my_world.reset()

    carb.log_info("=" * 80)
    carb.log_info("STARTING AUTOMATIC TESTS")
    carb.log_info("=" * 80)

    # Test commands
    test_commands = [
        {"name": "FORWARD (W)", "cmd": [0.4, 0.0, 0.0], "swap": [0.0, 0.4, 0.0]},
        {"name": "BACKWARD (S)", "cmd": [-0.4, 0.0, 0.0], "swap": [0.0, -0.4, 0.0]},
        {"name": "STRAFE LEFT (A)", "cmd": [0.0, 0.4, 0.0], "swap": [0.4, 0.0, 0.0]},
        {"name": "STRAFE RIGHT (D)", "cmd": [0.0, -0.4, 0.0], "swap": [-0.4, 0.0, 0.0]},
        {"name": "ROTATE CCW (Q)", "cmd": [0.0, 0.0, 0.5], "swap": [0.0, 0.0, -0.5]},
        {"name": "ROTATE CW (E)", "cmd": [0.0, 0.0, -0.5], "swap": [0.0, 0.0, 0.5]},
    ]

    step_count = 0
    test_index = 0
    test_duration = 100  # frames per test
    wait_duration = 50   # frames between tests

    in_test = False
    current_test = None

    try:
        while simulation_app.is_running() and test_index < len(test_commands) * 2:
            my_world.step(render=True)

            if my_world.is_playing():
                # Determine which test to run
                test_cycle = step_count % (test_duration + wait_duration)

                if test_cycle == 0:
                    # Start new test
                    if test_index < len(test_commands):
                        current_test = test_commands[test_index]
                        in_test = True
                        carb.log_info("")
                        carb.log_info("=" * 80)
                        carb.log_info(f"TEST {test_index + 1}: {current_test['name']}")
                        carb.log_info("=" * 80)
                    else:
                        # Second round with swapped commands
                        idx = test_index - len(test_commands)
                        current_test = test_commands[idx]
                        in_test = True
                        carb.log_info("")
                        carb.log_info("=" * 80)
                        carb.log_info(f"TEST {test_index + 1}: {current_test['name']} (SWAPPED)")
                        carb.log_info("=" * 80)

                    test_index += 1

                elif test_cycle == test_duration:
                    # End test
                    in_test = False
                    carb.log_info("Test complete. Waiting...")

                # Apply command
                if in_test and current_test:
                    if test_index <= len(test_commands):
                        # First round: no swap
                        command_to_apply = current_test['cmd']
                        carb.log_info(f"Command (NO SWAP): {command_to_apply}")
                    else:
                        # Second round: with swap
                        command_to_apply = current_test['swap']
                        carb.log_info(f"Command (SWAPPED): {command_to_apply}")

                    wheel_actions = my_controller.forward(command=command_to_apply)

                    # Print wheel velocities every 20 frames
                    if test_cycle % 20 == 10 and wheel_actions.joint_velocities is not None:
                        vels = wheel_actions.joint_velocities
                        if len(vels) >= 4:
                            carb.log_info(f"  Wheel Velocities: FL={vels[0]:+6.2f} FR={vels[1]:+6.2f} RL={vels[2]:+6.2f} RR={vels[3]:+6.2f} rad/s")

                    my_lukebot.apply_wheel_actions(wheel_actions)
                else:
                    # Stop
                    wheel_actions = my_controller.forward(command=[0.0, 0.0, 0.0])
                    my_lukebot.apply_wheel_actions(wheel_actions)

                step_count += 1

    finally:
        carb.log_info("=" * 80)
        carb.log_info("TESTS COMPLETE")
        carb.log_info("=" * 80)
        simulation_app.close()


if __name__ == "__main__":
    main()
