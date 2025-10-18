# Test direct velocity control bypassing WheeledRobot.apply_wheel_actions

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import carb
import numpy as np
import omni.kit.commands
from isaacsim.core.api import World
from isaacsim.core.utils.stage import get_current_stage
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from pxr import UsdPhysics

def main():
    my_world = World(stage_units_in_meters=1.0)
    my_world.scene.add_default_ground_plane()
    set_camera_view(eye=[2.0, 2.0, 2.0], target=[0.0, 0.0, 0.2], camera_prim_path="/OmniverseKit_Persp")

    urdf_path = "C:/Users/Luke/Desktop/issac_sim/source/extensions/isaacsim.asset.importer.urdf/data/urdf/robots/lukebot/urdf/lukebot.urdf"

    carb.log_info("=" * 80)
    carb.log_info("DIRECT VELOCITY CONTROL TEST")
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

    wheel_dof_names = [
        "front_left_wheel_joint",
        "front_right_wheel_joint",
        "rear_left_wheel_joint",
        "rear_right_wheel_joint",
    ]

    # Set drive properties for velocity control
    for joint_name in wheel_dof_names:
        joint_path = f"{robot_prim_path}/joints/{joint_name}"
        joint_prim = stage.GetPrimAtPath(joint_path)
        if joint_prim.IsValid():
            drive_api = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
            drive_api.GetTypeAttr().Set("velocity")  # IMPORTANT: Set to velocity mode!
            drive_api.GetDampingAttr().Set(1000.0)  # High damping for velocity control
            drive_api.GetStiffnessAttr().Set(0.0)    # No stiffness for velocity control
            drive_api.GetMaxForceAttr().Set(1000.0)  # Max torque
            carb.log_info(f"Configured {joint_name} for velocity control")

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

    carb.log_info("\nTesting DIRECT velocity targets...")

    test_sequence = [
        {"name": "ALL ZERO", "velocities": [0.0, 0.0, 0.0, 0.0], "duration": 100},
        {"name": "FL only (index 0)", "velocities": [5.0, 0.0, 0.0, 0.0], "duration": 200},
        {"name": "STOP", "velocities": [0.0, 0.0, 0.0, 0.0], "duration": 100},
        {"name": "FR only (index 1)", "velocities": [0.0, 5.0, 0.0, 0.0], "duration": 200},
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
                        carb.log_info(f"    Target velocities: {current_test['velocities']}")

                    # Use set_joint_velocity_targets DIRECTLY
                    # Note: This sets velocities for ALL DOFs, not just wheels
                    my_lukebot.set_joint_velocity_targets(
                        velocities=np.array(current_test["velocities"]),
                        indices=np.array([0, 1, 2, 3])  # Wheel indices
                    )

                    # Log actual velocities
                    if elapsed % 30 == 0 and elapsed > 0:
                        actual_vels = my_lukebot.get_joint_velocities()
                        carb.log_info(f"    Actual velocities: {actual_vels}")

                    if elapsed >= current_test["duration"]:
                        test_index += 1
                        test_start_step = step_count + 1

                else:
                    my_lukebot.set_joint_velocity_targets(
                        velocities=np.array([0.0, 0.0, 0.0, 0.0]),
                        indices=np.array([0, 1, 2, 3])
                    )

                    if step_count % 120 == 0:
                        carb.log_info("\n" + "=" * 80)
                        carb.log_info("TEST COMPLETE - Check if individual wheels worked!")
                        carb.log_info("=" * 80)

                step_count += 1

    finally:
        simulation_app.close()


if __name__ == "__main__":
    main()
