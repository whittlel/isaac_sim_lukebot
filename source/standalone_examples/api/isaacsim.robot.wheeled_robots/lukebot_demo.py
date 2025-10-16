# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Lukebot Mecanum Wheel Robot Demo
A mecanum wheel robot with OAK-D IOT 75 depth camera for testing in different scenes
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
    carb.log_info("Importing Lukebot URDF...")
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

    # Add mecanum wheel attributes to wheel joints
    wheel_joints = [
        "front_left_wheel_joint",
        "front_right_wheel_joint",
        "rear_left_wheel_joint",
        "rear_right_wheel_joint",
    ]

    # Mecanum wheel radius (50mm = 0.05m)
    wheel_radius = 0.050

    # Mecanum angles: +45 for left wheels, -45 for right wheels (in radians)
    mecanum_angles = [np.pi / 4, -np.pi / 4, -np.pi / 4, np.pi / 4]

    carb.log_info("Configuring mecanum wheels...")
    for joint_name, angle in zip(wheel_joints, mecanum_angles):
        joint_path = f"{robot_prim_path}/joints/{joint_name}"
        joint_prim = stage.GetPrimAtPath(joint_path)
        if joint_prim.IsValid():
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
            carb.log_warn(f"  ✗ Joint not found: {joint_path}")

    # Setup OAK-D camera (it already exists from URDF)
    camera_link_path = f"{robot_prim_path}/links/oak_d_camera"
    camera_prim = stage.GetPrimAtPath(camera_link_path)

    if camera_prim.IsValid():
        carb.log_info(f"Setting up OAK-D IOT 75 camera at: {camera_link_path}")

        # The camera prim already exists, now we initialize it as a Camera sensor
        camera = Camera(
            prim_path=camera_link_path,
            resolution=(1280, 800),
            frequency=30,
            name="oak_d_camera",
        )

        camera.initialize()

        # Set camera properties to match OAK-D IOT 75
        camera_api = UsdGeom.Camera.Get(stage, camera_link_path)
        if camera_api:
            # Set horizontal FOV to 75 degrees
            horizontal_fov_deg = 75.0
            horizontal_aperture = 20.955  # mm
            focal_length = horizontal_aperture / (2 * np.tan(np.radians(horizontal_fov_deg) / 2))

            camera_api.GetFocalLengthAttr().Set(focal_length)
            camera_api.GetHorizontalApertureAttr().Set(horizontal_aperture)
            camera_api.GetClippingRangeAttr().Set(Gf.Vec2f(0.35, 10.0))
            carb.log_info("  ✓ Camera configured with 75° FOV, 1280x800 resolution")
    else:
        carb.log_warn(f"Camera link not found at: {camera_link_path}")
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

    # Reset world
    my_world.reset()

    carb.log_info("=" * 80)
    carb.log_info("LUKEBOT ROBOT SUCCESSFULLY INITIALIZED!")
    carb.log_info("=" * 80)
    carb.log_info(f"Robot location: {robot_prim_path}")
    carb.log_info("Robot specifications:")
    carb.log_info("  - Chassis: 420mm x 330mm mecanum wheel platform")
    carb.log_info("  - Wheels: 4x 100mm diameter mecanum wheels")
    carb.log_info("  - Camera: OAK-D IOT 75 (1280x800, 75° FOV, 0.35-10m range)")
    carb.log_info("  - IMU: Onboard inertial measurement unit")
    carb.log_info("")
    carb.log_info("Capabilities:")
    carb.log_info("  - Omnidirectional movement (holonomic drive)")
    carb.log_info("  - Real-time depth perception")
    carb.log_info("  - Motion sensing")
    carb.log_info("=" * 80)
    carb.log_info("The simulation is now running.")
    carb.log_info("You can:")
    carb.log_info("  1. Use the UI to manually move the robot")
    carb.log_info("  2. Write controller code to autonomously navigate")
    carb.log_info("  3. Modify the scene to test different environments")
    carb.log_info("  4. Access camera data for vision processing")
    carb.log_info("=" * 80)

    step_count = 0

    while simulation_app.is_running():
        my_world.step(render=True)

        if my_world.is_playing():
            # Log camera data periodically
            if camera and step_count % 200 == 0 and step_count > 0:
                try:
                    camera_data = camera.get_current_frame()
                    if camera_data is not None:
                        carb.log_info(f"[Frame {step_count}] Camera data available: {list(camera_data.keys())}")
                except Exception as e:
                    pass  # Camera might not be ready yet

            step_count += 1

    simulation_app.close()


if __name__ == "__main__":
    main()
