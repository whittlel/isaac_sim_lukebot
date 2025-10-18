# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Lukebot Wheel Debug - Check what joints actually exist
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

    # Set camera view
    set_camera_view(eye=[2.0, 2.0, 2.0], target=[0.0, 0.0, 0.2], camera_prim_path="/OmniverseKit_Persp")

    # Get URDF path
    urdf_path = "C:/Users/Luke/Desktop/issac_sim/source/extensions/isaacsim.asset.importer.urdf/data/urdf/robots/lukebot/urdf/lukebot.urdf"

    carb.log_info("=" * 80)
    carb.log_info("LUKEBOT WHEEL DEBUG - Checking Joint Configuration")
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
            carb.log_info(f"\nFound lukebot at: {robot_prim_path}")
            break

    if robot_prim is None or not robot_prim.IsValid():
        carb.log_error("Failed to find lukebot robot after import")
        simulation_app.close()
        return

    # List ALL joints in the robot
    carb.log_info("\n" + "=" * 80)
    carb.log_info("ALL PRIMS IN ROBOT:")
    carb.log_info("=" * 80)
    for prim in robot_prim.GetChildren():
        prim_type = prim.GetTypeName()
        prim_name = prim.GetName()
        carb.log_info(f"  {prim_name} ({prim_type})")

        # If it's a joint, show more details
        if "Joint" in prim_type:
            joint = UsdPhysics.Joint(prim)
            if joint:
                carb.log_info(f"    -> Body0: {joint.GetBody0Rel().GetTargets()}")
                carb.log_info(f"    -> Body1: {joint.GetBody1Rel().GetTargets()}")

    wheel_dof_names = [
        "front_left_wheel_joint",
        "front_right_wheel_joint",
        "rear_left_wheel_joint",
        "rear_right_wheel_joint",
    ]

    carb.log_info("\n" + "=" * 80)
    carb.log_info("CHECKING WHEEL DOF NAMES:")
    carb.log_info("=" * 80)
    for i, joint_name in enumerate(wheel_dof_names):
        possible_paths = [
            f"{robot_prim_path}/{joint_name}",
            f"{robot_prim_path}/joints/{joint_name}",
        ]

        found = False
        for joint_path in possible_paths:
            joint_prim = stage.GetPrimAtPath(joint_path)
            if joint_prim.IsValid():
                carb.log_info(f"  ✓ [{i}] {joint_name} FOUND at {joint_path}")
                found = True

                # Get joint details
                joint = UsdPhysics.Joint(joint_prim)
                if joint:
                    body0 = joint.GetBody0Rel().GetTargets()
                    body1 = joint.GetBody1Rel().GetTargets()
                    carb.log_info(f"      Parent: {body0}")
                    carb.log_info(f"      Child: {body1}")
                break

        if not found:
            carb.log_error(f"  ✗ [{i}] {joint_name} NOT FOUND!")

    # Add robot to scene
    my_lukebot = my_world.scene.add(
        WheeledRobot(
            prim_path=robot_prim_path,
            name="my_lukebot",
            wheel_dof_names=wheel_dof_names,
            create_robot=False,
            position=np.array([0, 0.0, 0.15]),
        )
    )

    my_world.reset()

    # Check what DOFs the articulation actually has
    carb.log_info("\n" + "=" * 80)
    carb.log_info("ARTICULATION DOF INFORMATION:")
    carb.log_info("=" * 80)

    dof_names = my_lukebot.dof_names
    carb.log_info(f"Number of DOFs: {len(dof_names)}")
    carb.log_info(f"DOF names: {dof_names}")

    num_dofs = my_lukebot.num_dof
    carb.log_info(f"num_dof property: {num_dofs}")

    # Get current DOF positions
    dof_positions = my_lukebot.get_joint_positions()
    carb.log_info(f"DOF positions: {dof_positions}")

    # Get current DOF velocities
    dof_velocities = my_lukebot.get_joint_velocities()
    carb.log_info(f"DOF velocities: {dof_velocities}")

    carb.log_info("\n" + "=" * 80)
    carb.log_info("WHEEL DOF INDICES:")
    carb.log_info("=" * 80)
    wheel_indices = my_lukebot._wheel_dof_indices
    carb.log_info(f"Wheel indices array: {wheel_indices}")
    carb.log_info(f"Expected: [0, 1, 2, 3] for 4 wheels")

    # Now test spinning wheels with detailed logging
    carb.log_info("\n" + "=" * 80)
    carb.log_info("TESTING WHEEL COMMANDS:")
    carb.log_info("=" * 80)

    test_sequence = [
        {"name": "ALL ZERO", "velocities": [0.0, 0.0, 0.0, 0.0], "duration": 100},
        {"name": "Index 0 only", "velocities": [5.0, 0.0, 0.0, 0.0], "duration": 150},
        {"name": "STOP", "velocities": [0.0, 0.0, 0.0, 0.0], "duration": 50},
        {"name": "Index 1 only", "velocities": [0.0, 5.0, 0.0, 0.0], "duration": 150},
        {"name": "STOP", "velocities": [0.0, 0.0, 0.0, 0.0], "duration": 50},
        {"name": "Index 2 only", "velocities": [0.0, 0.0, 5.0, 0.0], "duration": 150},
        {"name": "STOP", "velocities": [0.0, 0.0, 0.0, 0.0], "duration": 50},
        {"name": "Index 3 only", "velocities": [0.0, 0.0, 0.0, 5.0], "duration": 150},
        {"name": "STOP", "velocities": [0.0, 0.0, 0.0, 0.0], "duration": 100},
    ]

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

                if test_index < len(test_sequence):
                    current_test = test_sequence[test_index]
                    elapsed = step_count - test_start_step

                    if elapsed == 0:
                        carb.log_info(f"\n>>> TEST: {current_test['name']}")
                        carb.log_info(f"    Command velocities: {current_test['velocities']}")

                    # Apply wheel velocities
                    action = ArticulationAction(joint_velocities=current_test["velocities"])
                    my_lukebot.apply_wheel_actions(action)

                    # Log actual velocities every 30 frames
                    if elapsed % 30 == 0 and elapsed > 0:
                        actual_vels = my_lukebot.get_joint_velocities()
                        carb.log_info(f"    Actual DOF velocities: {actual_vels}")

                    if elapsed >= current_test["duration"]:
                        test_index += 1
                        test_start_step = step_count + 1

                else:
                    my_lukebot.apply_wheel_actions(ArticulationAction(joint_velocities=[0.0, 0.0, 0.0, 0.0]))

                    if step_count % 120 == 0:
                        carb.log_info("\n" + "=" * 80)
                        carb.log_info("TESTS COMPLETE!")
                        carb.log_info("Check the log output above for DOF information.")
                        carb.log_info("=" * 80 + "\n")

                step_count += 1

    finally:
        simulation_app.close()


if __name__ == "__main__":
    main()
