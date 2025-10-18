# Test using Articulation class directly instead of WheeledRobot

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import carb
import numpy as np
import omni.kit.commands
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.stage import get_current_stage
from isaacsim.core.utils.viewports import set_camera_view
from pxr import UsdPhysics

def main():
    my_world = World(stage_units_in_meters=1.0)
    my_world.scene.add_default_ground_plane()
    set_camera_view(eye=[2.0, 2.0, 2.0], target=[0.0, 0.0, 0.2], camera_prim_path="/OmniverseKit_Persp")

    urdf_path = "C:/Users/Luke/Desktop/issac_sim/source/extensions/isaacsim.asset.importer.urdf/data/urdf/robots/lukebot/urdf/lukebot.urdf"

    carb.log_info("=" * 80)
    carb.log_info("ROBOT CLASS TEST (not WheeledRobot)")
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

    omni.kit.commands.execute("URDFParseAndImportFile", urdf_path=urdf_path, import_config=import_config)

    stage = get_current_stage()
    robot_prim_path = "/lukebot"

    wheel_joint_names = [
        "front_left_wheel_joint",
        "front_right_wheel_joint",
        "rear_left_wheel_joint",
        "rear_right_wheel_joint",
    ]

    # Configure drive mode for ALL wheels
    for joint_name in wheel_joint_names:
        joint_path = f"{robot_prim_path}/joints/{joint_name}"
        joint_prim = stage.GetPrimAtPath(joint_path)
        if joint_prim.IsValid():
            drive_api = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
            drive_api.GetDampingAttr().Set(100.0)
            drive_api.GetStiffnessAttr().Set(0.0)
            carb.log_info(f"Configured {joint_name}")

    # Use Robot class directly (NOT WheeledRobot)
    my_lukebot = my_world.scene.add(
        Robot(
            prim_path=robot_prim_path,
            name="my_lukebot",
            position=np.array([0, 0.0, 0.15]),
        )
    )

    my_world.reset()

    carb.log_info("\nRobot DOF info:")
    carb.log_info(f"  DOF names: {my_lukebot.dof_names}")
    carb.log_info(f"  Num DOFs: {my_lukebot.num_dof}")

    carb.log_info("\nTesting individual wheel control...")

    test_sequence = [
        {"name": "ALL ZERO", "velocities": np.array([0.0, 0.0, 0.0, 0.0]), "duration": 100},
        {"name": "FL ONLY (index 0)", "velocities": np.array([5.0, 0.0, 0.0, 0.0]), "duration": 200},
        {"name": "STOP", "velocities": np.array([0.0, 0.0, 0.0, 0.0]), "duration": 50},
        {"name": "FR ONLY (index 1)", "velocities": np.array([0.0, 5.0, 0.0, 0.0]), "duration": 200},
        {"name": "STOP", "velocities": np.array([0.0, 0.0, 0.0, 0.0]), "duration": 50},
        {"name": "RL ONLY (index 2)", "velocities": np.array([0.0, 0.0, 5.0, 0.0]), "duration": 200},
        {"name": "STOP", "velocities": np.array([0.0, 0.0, 0.0, 0.0]), "duration": 50},
        {"name": "RR ONLY (index 3)", "velocities": np.array([0.0, 0.0, 0.0, 5.0]), "duration": 200},
        {"name": "FINAL STOP", "velocities": np.array([0.0, 0.0, 0.0, 0.0]), "duration": 100},
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
                        carb.log_info(f"\n>>> TEST {test_index+1}/{len(test_sequence)}: {current_test['name']}")
                        carb.log_info(f"    Commanding: {current_test['velocities']}")

                    # Set velocity targets using Robot class method
                    my_lukebot.set_joint_velocity_targets(current_test["velocities"])

                    # Log actual velocities periodically
                    if elapsed % 30 == 0 and elapsed > 10:
                        actual_vels = my_lukebot.get_joint_velocities()
                        carb.log_info(f"    Actual velocities: {actual_vels}")

                    if elapsed >= current_test["duration"]:
                        test_index += 1
                        test_start_step = step_count + 1

                else:
                    # All tests complete
                    my_lukebot.set_joint_velocity_targets(np.array([0.0, 0.0, 0.0, 0.0]))

                    if step_count % 120 == 0:
                        carb.log_info("\n" + "=" * 80)
                        carb.log_info("ALL TESTS COMPLETE!")
                        carb.log_info("Did individual wheels spin correctly?")
                        carb.log_info("=" * 80)

                step_count += 1

    finally:
        simulation_app.close()


if __name__ == "__main__":
    main()
