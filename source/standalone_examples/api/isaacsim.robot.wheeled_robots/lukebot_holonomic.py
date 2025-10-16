# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Lukebot Holonomic Controller Demo
Demonstrates Lukebot with HolonomicController for omnidirectional movement
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
from isaacsim.sensors.camera import Camera
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
    carb.log_info("Importing Lukebot URDF with HolonomicController...")
    carb.log_info("=" * 80)

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

            # Set drive properties
            drive_api = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
            drive_api.GetDampingAttr().Set(100.0)
            drive_api.GetStiffnessAttr().Set(0.0)
            carb.log_info(f"  ✓ Configured wheel: {joint_name}")
        else:
            carb.log_warn(f"  ✗ Joint not found: {joint_name}")

    # Setup OAK-D camera - note: Camera sensor expects a Camera prim, not a link
    # We'll skip camera initialization here since it's complex with URDF imports
    # The camera link exists but needs to be converted to a proper Camera prim
    carb.log_info("Camera setup skipped for this demo (URDF link needs conversion to Camera prim)")
    camera = None

    # Create test obstacles
    carb.log_info("Creating test scene with obstacles...")

    # Box obstacle 1
    box1_prim = stage.DefinePrim("/World/Obstacles/Box1", "Cube")
    UsdGeom.Xform(box1_prim).AddTranslateOp().Set(Gf.Vec3f(1.5, 0.8, 0.25))
    UsdGeom.Xform(box1_prim).AddScaleOp().Set(Gf.Vec3f(0.4, 0.4, 0.5))
    box1_prim.GetAttribute("primvars:displayColor").Set([(0.9, 0.3, 0.3)])
    UsdPhysics.CollisionAPI.Apply(box1_prim)
    UsdPhysics.RigidBodyAPI.Apply(box1_prim)

    # Box obstacle 2
    box2_prim = stage.DefinePrim("/World/Obstacles/Box2", "Cube")
    UsdGeom.Xform(box2_prim).AddTranslateOp().Set(Gf.Vec3f(-1.2, -1.2, 0.2))
    UsdGeom.Xform(box2_prim).AddScaleOp().Set(Gf.Vec3f(0.3, 0.3, 0.4))
    box2_prim.GetAttribute("primvars:displayColor").Set([(0.3, 0.9, 0.3)])
    UsdPhysics.CollisionAPI.Apply(box2_prim)
    UsdPhysics.RigidBodyAPI.Apply(box2_prim)

    # Cylinder obstacle
    cylinder_prim = stage.DefinePrim("/World/Obstacles/Cylinder1", "Cylinder")
    UsdGeom.Xform(cylinder_prim).AddTranslateOp().Set(Gf.Vec3f(0.8, -1.5, 0.3))
    UsdGeom.Xform(cylinder_prim).AddScaleOp().Set(Gf.Vec3f(0.15, 0.15, 0.3))
    cylinder_prim.GetAttribute("primvars:displayColor").Set([(0.3, 0.3, 0.9)])
    UsdPhysics.CollisionAPI.Apply(cylinder_prim)
    UsdPhysics.RigidBodyAPI.Apply(cylinder_prim)

    # Wall
    wall_prim = stage.DefinePrim("/World/Obstacles/Wall1", "Cube")
    UsdGeom.Xform(wall_prim).AddTranslateOp().Set(Gf.Vec3f(0.0, 2.5, 0.4))
    UsdGeom.Xform(wall_prim).AddScaleOp().Set(Gf.Vec3f(3.0, 0.1, 0.8))
    wall_prim.GetAttribute("primvars:displayColor").Set([(0.6, 0.6, 0.6)])
    UsdPhysics.CollisionAPI.Apply(wall_prim)
    UsdPhysics.RigidBodyAPI.Apply(wall_prim)

    carb.log_info("  ✓ Created test obstacles")

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

    # Reset world
    my_world.reset()

    carb.log_info("=" * 80)
    carb.log_info("LUKEBOT HOLONOMIC CONTROLLER DEMO READY!")
    carb.log_info("=" * 80)
    carb.log_info(f"Robot location: {robot_prim_path}")
    carb.log_info("Robot specifications:")
    carb.log_info("  - Chassis: 420mm x 330mm mecanum wheel platform")
    carb.log_info("  - Wheels: 4x 100mm diameter mecanum wheels")
    carb.log_info("  - Camera: OAK-D IOT 75 (1280x800, 75° FOV, 0.35-10m range)")
    carb.log_info("  - Controller: HolonomicController for omnidirectional motion")
    carb.log_info("")
    carb.log_info("Movement pattern:")
    carb.log_info("  - Frames 0-500: Forward (X+)")
    carb.log_info("  - Frames 500-1000: Strafe left (Y+)")
    carb.log_info("  - Frames 1000-1500: Strafe right (Y-)")
    carb.log_info("  - Frames 1500-1700: Rotate CCW")
    carb.log_info("=" * 80)

    i = 0
    reset_needed = False

    while simulation_app.is_running():
        my_world.step(render=True)

        if my_world.is_stopped() and not reset_needed:
            reset_needed = True

        if my_world.is_playing():
            if reset_needed:
                my_world.reset()
                my_controller.reset()
                reset_needed = False
                i = 0

            # Demonstrate omnidirectional movement
            if i >= 0 and i < 500:
                # Move forward
                my_lukebot.apply_wheel_actions(my_controller.forward(command=[0.3, 0.0, 0.0]))
            elif i >= 500 and i < 1000:
                # Strafe left
                my_lukebot.apply_wheel_actions(my_controller.forward(command=[0.0, 0.3, 0.0]))
            elif i >= 1000 and i < 1500:
                # Strafe right
                my_lukebot.apply_wheel_actions(my_controller.forward(command=[0.0, -0.3, 0.0]))
            elif i >= 1500 and i < 1700:
                # Rotate in place
                my_lukebot.apply_wheel_actions(my_controller.forward(command=[0.0, 0.0, 0.5]))
            elif i == 1700:
                i = 0

            i += 1

    simulation_app.close()


if __name__ == "__main__":
    main()
