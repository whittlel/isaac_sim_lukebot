# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Lukebot Individual Wheel Test
Tests each wheel INDIVIDUALLY to verify wheel order and naming
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import carb
import numpy as np
import omni.kit.commands
from isaacsim.core.api import World
from isaacsim.core.utils.stage import get_current_stage
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from isaacsim.core.utils.types import ArticulationAction
from pxr import UsdPhysics

def main():
    # Create world
    my_world = World(stage_units_in_meters=1.0)
    my_world.scene.add_default_ground_plane()

    # Set camera view - viewing from above-front
    set_camera_view(eye=[2.0, 2.0, 2.0], target=[0.0, 0.0, 0.2], camera_prim_path="/OmniverseKit_Persp")

    # Get URDF path
    urdf_path = "C:/Users/Luke/Desktop/issac_sim/source/extensions/isaacsim.asset.importer.urdf/data/urdf/robots/lukebot/urdf/lukebot.urdf"

    carb.log_info("=" * 80)
    carb.log_info("LUKEBOT INDIVIDUAL WHEEL TEST")
    carb.log_info("=" * 80)
    carb.log_info("\nThis test spins each wheel individually to verify wheel order.")
    carb.log_info("Watch which wheel spins for each test!\n")

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

    carb.log_info("\nWheel DOF order:")
    for i, name in enumerate(wheel_dof_names):
        carb.log_info(f"  {i}: {name}")

    # Set drive properties for all wheels
    for joint_name in wheel_dof_names:
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
            drive_api = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
            drive_api.GetDampingAttr().Set(10.0)
            drive_api.GetStiffnessAttr().Set(0.0)

    # Add robot to scene
    my_lukebot = my_world.scene.add(
        WheeledRobot(
            prim_path=robot_prim_path,
            name="my_lukebot",
            wheel_dof_names=wheel_dof_names,
            create_robot=False,
            position=np.array([0, 0.0, 0.15]),  # Slightly higher
        )
    )

    my_world.reset()

    # Test sequence - spin each wheel individually
    wheel_speed = 5.0  # rad/s
    test_sequence = [
        {"name": "ALL STOPPED", "velocities": [0.0, 0.0, 0.0, 0.0], "duration": 100},
        {"name": "FRONT LEFT wheel (index 0)", "velocities": [wheel_speed, 0.0, 0.0, 0.0], "duration": 150},
        {"name": "STOP", "velocities": [0.0, 0.0, 0.0, 0.0], "duration": 100},
        {"name": "FRONT RIGHT wheel (index 1)", "velocities": [0.0, wheel_speed, 0.0, 0.0], "duration": 150},
        {"name": "STOP", "velocities": [0.0, 0.0, 0.0, 0.0], "duration": 100},
        {"name": "REAR LEFT wheel (index 2)", "velocities": [0.0, 0.0, wheel_speed, 0.0], "duration": 150},
        {"name": "STOP", "velocities": [0.0, 0.0, 0.0, 0.0], "duration": 100},
        {"name": "REAR RIGHT wheel (index 3)", "velocities": [0.0, 0.0, 0.0, wheel_speed], "duration": 150},
        {"name": "STOP", "velocities": [0.0, 0.0, 0.0, 0.0], "duration": 100},
        {"name": "ALL WHEELS FORWARD", "velocities": [wheel_speed, wheel_speed, wheel_speed, wheel_speed], "duration": 150},
        {"name": "STOP", "velocities": [0.0, 0.0, 0.0, 0.0], "duration": 100},
    ]

    carb.log_info("\n" + "=" * 80)
    carb.log_info("TEST SEQUENCE:")
    carb.log_info("=" * 80)
    for i, test in enumerate(test_sequence):
        carb.log_info(f"{i+1}. {test['name']}")
        carb.log_info(f"   Velocities: {test['velocities']}")
    carb.log_info("=" * 80 + "\n")

    carb.log_info("INSTRUCTIONS:")
    carb.log_info("  Watch the robot carefully!")
    carb.log_info("  Note which physical wheel spins for each test.")
    carb.log_info("  The wheel order should be:")
    carb.log_info("    0: Front Left (forward-left corner)")
    carb.log_info("    1: Front Right (forward-right corner)")
    carb.log_info("    2: Rear Left (back-left corner)")
    carb.log_info("    3: Rear Right (back-right corner)")
    carb.log_info("\n" + "=" * 80 + "\n")

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
                    reset_needed = False
                    step_count = 0
                    test_index = 0
                    test_start_step = 0

                # Execute test sequence
                if test_index < len(test_sequence):
                    current_test = test_sequence[test_index]
                    elapsed = step_count - test_start_step

                    if elapsed == 0:
                        carb.log_info(f"\n{'='*60}")
                        carb.log_info(f">>> TEST {test_index + 1}/{len(test_sequence)}: {current_test['name']}")
                        carb.log_info(f"    Wheel velocities [FL, FR, RL, RR]: {current_test['velocities']}")
                        carb.log_info(f"{'='*60}")

                    # Apply wheel velocities directly
                    action = ArticulationAction(joint_velocities=current_test["velocities"])
                    my_lukebot.apply_wheel_actions(action)

                    if elapsed >= current_test["duration"]:
                        test_index += 1
                        test_start_step = step_count + 1

                else:
                    # All tests complete
                    my_lukebot.apply_wheel_actions(ArticulationAction(joint_velocities=[0.0, 0.0, 0.0, 0.0]))

                    if step_count % 120 == 0:
                        carb.log_info("\n" + "=" * 80)
                        carb.log_info("ALL TESTS COMPLETE!")
                        carb.log_info("=" * 80)
                        carb.log_info("\nDid each wheel spin in the correct position?")
                        carb.log_info("If not, note which wheel actually spun for each test.")
                        carb.log_info("\nPress STOP to end test.")
                        carb.log_info("=" * 80 + "\n")

                step_count += 1

    finally:
        simulation_app.close()


if __name__ == "__main__":
    main()
