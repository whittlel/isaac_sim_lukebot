# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Lukebot with OAK-D IOT 75 Camera Integration
Demonstrates proper camera setup and depth/RGB data acquisition for autonomous navigation
Uses new motion controller with physics bypass for reliable movement
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import carb
import numpy as np
import omni.kit.commands
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import get_current_stage
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.sensors.camera import Camera
from pxr import Gf, UsdGeom, UsdPhysics

# Import our motion controller
from lukebot_motion_controller import create_motion_controller

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
    carb.log_info("LUKEBOT OAK-D CAMERA DEMO")
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

    # Configure wheel drive properties for visual wheel spinning
    wheel_dof_names = [
        "front_left_wheel_joint",
        "front_right_wheel_joint",
        "rear_left_wheel_joint",
        "rear_right_wheel_joint",
    ]

    carb.log_info("Configuring wheel drive properties...")
    for joint_name in wheel_dof_names:
        joint_path = f"{robot_prim_path}/joints/{joint_name}"
        joint_prim = stage.GetPrimAtPath(joint_path)

        if joint_prim and joint_prim.IsValid():
            drive_api = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
            drive_api.GetDampingAttr().Set(100.0)
            drive_api.GetStiffnessAttr().Set(0.0)
            carb.log_info(f"  ✓ Configured {joint_name}")
        else:
            carb.log_warn(f"  ✗ Joint not found: {joint_name}")

    # Create OAK-D IOT 75 Camera
    # Note: We create a separate Camera prim instead of trying to use the URDF link
    carb.log_info("Setting up OAK-D IOT 75 camera...")

    # Camera should be positioned at the front of the robot, elevated
    # Based on URDF: camera_mount at (0.21, 0, 0.05) + camera offset (0.025, 0, 0.04)
    camera_position_local = np.array([0.235, 0.0, 0.09])  # Relative to chassis

    # Create camera with OAK-D IOT 75 specifications
    oak_d_camera = Camera(
        prim_path="/World/Lukebot_Camera",
        position=camera_position_local,  # Will be adjusted after robot is added
        frequency=30,
        resolution=(1280, 800),
        orientation=np.array([1.0, 0.0, 0.0, 0.0]),  # Forward facing
    )

    # Create test scene with colored obstacles
    carb.log_info("Creating test scene with target cube...")

    # TARGET CUBE (Yellow - for the robot to find)
    target_cube = stage.DefinePrim("/World/Obstacles/TargetCube", "Cube")
    UsdGeom.Xform(target_cube).AddTranslateOp().Set(Gf.Vec3f(2.0, 0.0, 0.25))
    UsdGeom.Xform(target_cube).AddScaleOp().Set(Gf.Vec3f(0.5, 0.5, 0.5))
    target_cube.GetAttribute("primvars:displayColor").Set([(1.0, 1.0, 0.0)])  # Yellow
    UsdPhysics.CollisionAPI.Apply(target_cube)
    UsdPhysics.RigidBodyAPI.Apply(target_cube)

    # Red obstacle
    box1_prim = stage.DefinePrim("/World/Obstacles/Box1", "Cube")
    UsdGeom.Xform(box1_prim).AddTranslateOp().Set(Gf.Vec3f(1.5, -1.0, 0.25))
    UsdGeom.Xform(box1_prim).AddScaleOp().Set(Gf.Vec3f(0.4, 0.4, 0.5))
    box1_prim.GetAttribute("primvars:displayColor").Set([(0.9, 0.3, 0.3)])
    UsdPhysics.CollisionAPI.Apply(box1_prim)
    UsdPhysics.RigidBodyAPI.Apply(box1_prim)

    # Green obstacle
    box2_prim = stage.DefinePrim("/World/Obstacles/Box2", "Cube")
    UsdGeom.Xform(box2_prim).AddTranslateOp().Set(Gf.Vec3f(-1.2, 1.0, 0.2))
    UsdGeom.Xform(box2_prim).AddScaleOp().Set(Gf.Vec3f(0.3, 0.3, 0.4))
    box2_prim.GetAttribute("primvars:displayColor").Set([(0.3, 0.9, 0.3)])
    UsdPhysics.CollisionAPI.Apply(box2_prim)
    UsdPhysics.RigidBodyAPI.Apply(box2_prim)

    # Blue obstacle
    cylinder_prim = stage.DefinePrim("/World/Obstacles/Cylinder1", "Cylinder")
    UsdGeom.Xform(cylinder_prim).AddTranslateOp().Set(Gf.Vec3f(1.0, 1.5, 0.3))
    UsdGeom.Xform(cylinder_prim).AddScaleOp().Set(Gf.Vec3f(0.15, 0.15, 0.3))
    cylinder_prim.GetAttribute("primvars:displayColor").Set([(0.3, 0.3, 0.9)])
    UsdPhysics.CollisionAPI.Apply(cylinder_prim)
    UsdPhysics.RigidBodyAPI.Apply(cylinder_prim)

    carb.log_info("  ✓ Created test scene")

    # Add Lukebot as a Robot (NOT WheeledRobot)
    my_lukebot = my_world.scene.add(
        Robot(
            prim_path=robot_prim_path,
            name="my_lukebot",
            position=np.array([0, 0.0, 0.1]),
        )
    )

    # Create motion controller with actual Lukebot dimensions
    carb.log_info("Creating motion controller...")
    motion_controller = create_motion_controller(
        robot=my_lukebot,
        use_simulation=True,
        wheel_base=0.30,      # 300mm between front/rear axles
        track_width=0.34,     # 340mm between left/right wheels
        wheel_radius=0.05     # 50mm wheel radius
    )
    carb.log_info("  ✓ Motion controller initialized with physics bypass")

    # Reset world
    my_world.reset()

    # Initialize camera AFTER world reset
    oak_d_camera.initialize()

    # Set OAK-D IOT 75 camera properties
    # 75° horizontal FOV, 1280x800 resolution
    horizontal_fov_deg = 75.0
    horizontal_aperture = 20.955  # mm
    focal_length = horizontal_aperture / (2 * np.tan(np.radians(horizontal_fov_deg) / 2))

    oak_d_camera.set_focal_length(focal_length)
    oak_d_camera.set_horizontal_aperture(horizontal_aperture)
    oak_d_camera.set_vertical_aperture(horizontal_aperture * (800.0 / 1280.0))  # Maintain aspect ratio
    oak_d_camera.set_clipping_range(0.35, 10.0)  # OAK-D range: 0.35m - 10m

    # Add depth annotator AFTER camera initialization
    oak_d_camera.add_distance_to_image_plane_to_frame()

    carb.log_info("  ✓ OAK-D camera configured")

    carb.log_info("=" * 80)
    carb.log_info("LUKEBOT CAMERA DEMO READY!")
    carb.log_info("=" * 80)
    carb.log_info(f"Robot location: {robot_prim_path}")
    carb.log_info("Camera specifications:")
    carb.log_info("  - Model: OAK-D IOT 75")
    carb.log_info("  - Resolution: 1280x800")
    carb.log_info("  - Horizontal FOV: 75°")
    carb.log_info("  - Depth Range: 0.35m - 10m")
    carb.log_info("  - Frame Rate: 30 FPS")
    carb.log_info("")
    carb.log_info("Demonstration:")
    carb.log_info("  - Robot will move forward slowly")
    carb.log_info("  - Camera RGB and depth data logged every 60 frames")
    carb.log_info("  - Yellow cube at (2.0, 0.0, 0.25) is the target")
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
                motion_controller.reset()
                reset_needed = False
                i = 0

            # Move forward slowly to test camera
            motion_controller.set_velocity(vx=0.1, vy=0.0, omega=0.0)
            motion_controller.update(dt=1.0/60.0)  # 60 FPS

            # Log camera data periodically
            if i % 60 == 0 and i > 0:
                try:
                    # Get RGB data
                    rgb_data = oak_d_camera.get_rgb()

                    # Get current frame with all annotators
                    current_frame = oak_d_camera.get_current_frame()

                    if rgb_data is not None:
                        carb.log_info(f"[Frame {i}] RGB shape: {rgb_data.shape}, dtype: {rgb_data.dtype}")

                        # Analyze image for yellow pixels (target detection)
                        # Yellow in RGB: high R, high G, low B
                        if rgb_data.size > 0:
                            # Simple yellow detection
                            yellow_mask = (rgb_data[:,:,0] > 200) & (rgb_data[:,:,1] > 200) & (rgb_data[:,:,2] < 100)
                            yellow_pixels = np.sum(yellow_mask)
                            if yellow_pixels > 100:  # Threshold for detection
                                carb.log_info(f"  → Yellow target detected! ({yellow_pixels} pixels)")

                    # Get depth data from frame
                    if "distance_to_image_plane" in current_frame:
                        depth_data = current_frame["distance_to_image_plane"]
                        if hasattr(depth_data, 'shape') and depth_data.size > 0:
                            # Get min/max/mean depth
                            valid_depth = depth_data[np.isfinite(depth_data) & (depth_data > 0.01)]  # Filter out invalid/inf readings
                            if len(valid_depth) > 0:
                                min_dist = np.min(valid_depth)
                                mean_dist = np.mean(valid_depth)
                                max_dist = np.max(valid_depth)
                                carb.log_info(f"  → Depth - Min: {min_dist:.2f}m, Mean: {mean_dist:.2f}m, Max: {max_dist:.2f}m")
                            else:
                                carb.log_info(f"  → Depth data shape: {depth_data.shape}, but all values are invalid/inf")

                except Exception as e:
                    carb.log_warn(f"Camera data error: {e}")

            i += 1

    simulation_app.close()


if __name__ == "__main__":
    main()
