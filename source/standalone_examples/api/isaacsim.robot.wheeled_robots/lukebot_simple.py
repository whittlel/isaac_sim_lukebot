# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Lukebot Mecanum Wheel Robot Example (Simplified Version)
This script creates a lukebot robot directly in USD with:
- Mecanum wheel chassis
- OAK-D IOT 75 depth camera sensor
- Test scene for robot navigation
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import carb
import numpy as np
import omni.kit.commands
from isaacsim.core.api import World
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.stage import get_current_stage
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.sensors.camera import Camera
from omni.isaac.sensor import IMUSensor
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

# Enable necessary extensions
enable_extension("omni.isaac.sensor")


def main():
    # Create world
    my_world = World(stage_units_in_meters=1.0)
    my_world.scene.add_default_ground_plane()

    # Set camera view
    set_camera_view(eye=[3.0, 3.0, 2.0], target=[0.0, 0.0, 0.5], camera_prim_path="/OmniverseKit_Persp")

    # Get URDF path
    urdf_path = "C:/Users/Luke/Desktop/issac_sim/source/extensions/isaacsim.asset.importer.urdf/data/urdf/robots/lukebot/urdf/lukebot.urdf"
    robot_prim_path = "/World/Lukebot"

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
    result = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=urdf_path,
        import_config=import_config,
    )

    carb.log_info(f"URDF Import result: {result}")

    stage = get_current_stage()

    # Find the imported robot (it may have a different path)
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
        f"{robot_prim_path}/front_left_wheel_joint",
        f"{robot_prim_path}/front_right_wheel_joint",
        f"{robot_prim_path}/rear_left_wheel_joint",
        f"{robot_prim_path}/rear_right_wheel_joint",
    ]

    # Mecanum wheel radius (50mm = 0.05m)
    wheel_radius = 0.050

    # Mecanum angles: +45 for left wheels, -45 for right wheels (in radians)
    mecanum_angles = [np.pi / 4, -np.pi / 4, -np.pi / 4, np.pi / 4]

    for joint_path, angle in zip(wheel_joints, mecanum_angles):
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
            carb.log_info(f"Configured wheel: {joint_path}")

    # Setup OAK-D camera
    camera_prim_path = f"{robot_prim_path}/oak_d_camera"
    carb.log_info(f"Setting up camera at: {camera_prim_path}")

    camera = Camera(
        prim_path=camera_prim_path,
        resolution=(1280, 800),
        frequency=30,
        name="oak_d_camera",
    )

    camera.initialize()

    # Set camera properties to match OAK-D IOT 75
    camera_prim = stage.GetPrimAtPath(camera_prim_path)
    if camera_prim.IsValid():
        camera_api = UsdGeom.Camera(camera_prim)

        # Set horizontal FOV to 75 degrees
        horizontal_fov_deg = 75.0
        horizontal_aperture = 20.955  # mm
        focal_length = horizontal_aperture / (2 * np.tan(np.radians(horizontal_fov_deg) / 2))

        camera_api.GetFocalLengthAttr().Set(focal_length)
        camera_api.GetHorizontalApertureAttr().Set(horizontal_aperture)
        camera_api.GetClippingRangeAttr().Set(Gf.Vec2f(0.35, 10.0))

    # Add IMU sensor
    imu_prim_path = f"{robot_prim_path}/imu_link"
    imu_sensor = IMUSensor(
        prim_path=imu_prim_path,
        name="lukebot_imu",
        frequency=100,
        translation=np.array([0, 0, 0])
    )

    # Create test obstacles
    carb.log_info("Creating test scene...")

    # Box obstacle 1
    box1_prim = stage.DefinePrim("/World/Obstacles/Box1", "Cube")
    UsdGeom.Xform(box1_prim).AddTranslateOp().Set(Gf.Vec3f(2.0, 1.0, 0.25))
    UsdGeom.Xform(box1_prim).AddScaleOp().Set(Gf.Vec3f(0.5, 0.5, 0.5))
    box1_prim.GetAttribute("primvars:displayColor").Set([(0.8, 0.2, 0.2)])
    UsdPhysics.CollisionAPI.Apply(box1_prim)
    UsdPhysics.RigidBodyAPI.Apply(box1_prim)

    # Box obstacle 2
    box2_prim = stage.DefinePrim("/World/Obstacles/Box2", "Cube")
    UsdGeom.Xform(box2_prim).AddTranslateOp().Set(Gf.Vec3f(-1.5, -1.5, 0.25))
    UsdGeom.Xform(box2_prim).AddScaleOp().Set(Gf.Vec3f(0.3, 0.3, 0.5))
    box2_prim.GetAttribute("primvars:displayColor").Set([(0.2, 0.8, 0.2)])
    UsdPhysics.CollisionAPI.Apply(box2_prim)
    UsdPhysics.RigidBodyAPI.Apply(box2_prim)

    # Cylinder obstacle
    cylinder_prim = stage.DefinePrim("/World/Obstacles/Cylinder1", "Cylinder")
    UsdGeom.Xform(cylinder_prim).AddTranslateOp().Set(Gf.Vec3f(1.0, -2.0, 0.4))
    UsdGeom.Xform(cylinder_prim).AddScaleOp().Set(Gf.Vec3f(0.2, 0.2, 0.4))
    cylinder_prim.GetAttribute("primvars:displayColor").Set([(0.2, 0.2, 0.8)])
    UsdPhysics.CollisionAPI.Apply(cylinder_prim)
    UsdPhysics.RigidBodyAPI.Apply(cylinder_prim)

    # Reset world
    my_world.reset()
    imu_sensor.initialize()

    carb.log_info("=" * 80)
    carb.log_info("Lukebot initialized successfully!")
    carb.log_info(f"Robot path: {robot_prim_path}")
    carb.log_info("Camera resolution: 1280x800, FOV: 75 degrees")
    carb.log_info("Mecanum wheels configured for holonomic motion")
    carb.log_info("=" * 80)
    carb.log_info("Simulation is running. You can manually control the robot or close the window to exit.")
    carb.log_info("=" * 80)

    step_count = 0

    while simulation_app.is_running():
        my_world.step(render=True)

        if my_world.is_playing():
            # Read and log IMU data every 100 steps
            if step_count % 100 == 0:
                imu_data = imu_sensor.get_current_frame()
                if imu_data is not None and "lin_acc_x" in imu_data:
                    carb.log_info(
                        f"IMU - Linear acc: [{imu_data['lin_acc_x']:.3f}, {imu_data['lin_acc_y']:.3f}, {imu_data['lin_acc_z']:.3f}]"
                    )

            # Get camera data periodically
            if step_count % 200 == 0:
                camera_data = camera.get_current_frame()
                if camera_data is not None:
                    carb.log_info(f"Camera frame captured - Available data: {list(camera_data.keys())}")

            step_count += 1

    simulation_app.close()


if __name__ == "__main__":
    main()
