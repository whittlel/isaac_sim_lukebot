# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Lukebot with OAK-D IOT 75 Stereo Camera + ROS 2 Bridge for Isaac ROS vSLAM
Demonstrates complete stereo camera setup with ROS 2 topic publishing
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
import omni.graph.core as og

# Enable necessary extensions
enable_extension("omni.isaac.sensor")
enable_extension("omni.isaac.ros2_bridge")
enable_extension("omni.isaac.core_nodes")


def create_stereo_camera_graph(left_camera_path, right_camera_path):
    """
    Create OmniGraph action graph to publish stereo camera images to ROS 2

    This creates the necessary nodes to:
    1. Generate render products for both cameras
    2. Publish RGB images for left and right cameras
    3. Publish camera_info for both cameras

    Topics published:
    - /stereo_camera/left/image_raw
    - /stereo_camera/left/camera_info
    - /stereo_camera/right/image_raw
    - /stereo_camera/right/camera_info
    """

    try:
        carb.log_info("Creating ROS 2 stereo camera action graph...")

        # Create action graph
        (graph, nodes, _, _) = og.Controller.edit(
            {"graph_path": "/World/StereoCamera_Graph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("IsaacCreateRenderProductLeft", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                    ("IsaacCreateRenderProductRight", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                    ("ROS2CameraHelperLeft", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                    ("ROS2CameraHelperRight", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "IsaacCreateRenderProductLeft.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "IsaacCreateRenderProductRight.inputs:execIn"),
                    ("IsaacCreateRenderProductLeft.outputs:execOut", "ROS2CameraHelperLeft.inputs:execIn"),
                    ("IsaacCreateRenderProductRight.outputs:execOut", "ROS2CameraHelperRight.inputs:execIn"),
                    ("IsaacCreateRenderProductLeft.outputs:renderProductPath", "ROS2CameraHelperLeft.inputs:renderProductPath"),
                    ("IsaacCreateRenderProductRight.outputs:renderProductPath", "ROS2CameraHelperRight.inputs:renderProductPath"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    # Left camera configuration
                    ("IsaacCreateRenderProductLeft.inputs:cameraPrim", left_camera_path),
                    ("ROS2CameraHelperLeft.inputs:frameId", "camera_left"),
                    ("ROS2CameraHelperLeft.inputs:topicName", "stereo_camera/left"),
                    ("ROS2CameraHelperLeft.inputs:type", "rgb"),

                    # Right camera configuration
                    ("IsaacCreateRenderProductRight.inputs:cameraPrim", right_camera_path),
                    ("ROS2CameraHelperRight.inputs:frameId", "camera_right"),
                    ("ROS2CameraHelperRight.inputs:topicName", "stereo_camera/right"),
                    ("ROS2CameraHelperRight.inputs:type", "rgb"),
                ],
            },
        )

        carb.log_info("  ✓ ROS 2 stereo camera graph created")
        carb.log_info("  Topics configured:")
        carb.log_info("    - /stereo_camera/left/image_raw")
        carb.log_info("    - /stereo_camera/left/camera_info")
        carb.log_info("    - /stereo_camera/right/image_raw")
        carb.log_info("    - /stereo_camera/right/camera_info")

        return graph

    except Exception as e:
        carb.log_error(f"Failed to create stereo camera graph: {e}")
        return None


def main():
    # Create world
    my_world = World(stage_units_in_meters=1.0)
    my_world.scene.add_default_ground_plane()

    # Set camera view
    set_camera_view(eye=[3.0, 3.0, 2.5], target=[0.0, 0.0, 0.3], camera_prim_path="/OmniverseKit_Persp")

    # Get URDF path
    urdf_path = "C:/Users/Luke/Desktop/issac_sim/source/extensions/isaacsim.asset.importer.urdf/data/urdf/robots/lukebot/urdf/lukebot.urdf"

    carb.log_info("=" * 80)
    carb.log_info("LUKEBOT STEREO CAMERA + ROS 2 BRIDGE FOR vSLAM")
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

    # Add mecanum wheel attributes to wheel joints
    wheel_radius = 0.050  # 50mm = 0.05m
    mecanum_angles = [np.pi / 4, -np.pi / 4, -np.pi / 4, np.pi / 4]  # radians

    carb.log_info("Configuring mecanum wheels...")
    for joint_name, angle in zip(wheel_dof_names, mecanum_angles):
        # Try different possible paths
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

    # Create Stereo Camera Setup for OAK-D IOT 75
    carb.log_info("Setting up OAK-D IOT 75 stereo cameras...")

    # Camera mount position (front of robot, elevated)
    camera_center_position = np.array([0.235, 0.0, 0.09])  # Center between stereo pair

    # OAK-D IOT 75 baseline: 7.5cm between cameras
    baseline = 0.075  # meters
    half_baseline = baseline / 2.0

    # Left camera position (baseline/2 to the left in Y axis)
    left_camera_position = camera_center_position + np.array([0.0, half_baseline, 0.0])

    # Right camera position (baseline/2 to the right in Y axis)
    right_camera_position = camera_center_position + np.array([0.0, -half_baseline, 0.0])

    # Camera specifications
    resolution = (1280, 800)
    frequency = 30  # FPS
    horizontal_fov_deg = 75.0

    # Calculate focal length for 75° FOV
    horizontal_aperture = 20.955  # mm
    focal_length = horizontal_aperture / (2 * np.tan(np.radians(horizontal_fov_deg) / 2))
    vertical_aperture = horizontal_aperture * (800.0 / 1280.0)  # Maintain aspect ratio

    # Define camera prim paths
    left_camera_prim_path = "/World/Lukebot_Camera_Left"
    right_camera_prim_path = "/World/Lukebot_Camera_Right"

    # Create LEFT camera
    carb.log_info("  Creating left stereo camera...")
    left_camera = Camera(
        prim_path=left_camera_prim_path,
        position=left_camera_position,
        frequency=frequency,
        resolution=resolution,
        orientation=np.array([1.0, 0.0, 0.0, 0.0]),  # Forward facing
    )

    # Create RIGHT camera
    carb.log_info("  Creating right stereo camera...")
    right_camera = Camera(
        prim_path=right_camera_prim_path,
        position=right_camera_position,
        frequency=frequency,
        resolution=resolution,
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

    # Add walls for more depth variation
    wall1_prim = stage.DefinePrim("/World/Obstacles/Wall1", "Cube")
    UsdGeom.Xform(wall1_prim).AddTranslateOp().Set(Gf.Vec3f(3.0, -2.0, 0.5))
    UsdGeom.Xform(wall1_prim).AddScaleOp().Set(Gf.Vec3f(0.1, 2.0, 1.0))
    wall1_prim.GetAttribute("primvars:displayColor").Set([(0.7, 0.7, 0.7)])
    UsdPhysics.CollisionAPI.Apply(wall1_prim)
    UsdPhysics.RigidBodyAPI.Apply(wall1_prim)

    carb.log_info("  ✓ Created test scene")

    # Add Lukebot as a WheeledRobot
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

    # Initialize cameras AFTER world reset
    carb.log_info("Initializing stereo cameras...")
    left_camera.initialize()
    right_camera.initialize()

    # Configure LEFT camera properties
    left_camera.set_focal_length(focal_length)
    left_camera.set_horizontal_aperture(horizontal_aperture)
    left_camera.set_vertical_aperture(vertical_aperture)
    left_camera.set_clipping_range(0.35, 10.0)  # OAK-D range: 0.35m - 10m

    # Configure RIGHT camera properties
    right_camera.set_focal_length(focal_length)
    right_camera.set_horizontal_aperture(horizontal_aperture)
    right_camera.set_vertical_aperture(vertical_aperture)
    right_camera.set_clipping_range(0.35, 10.0)  # OAK-D range: 0.35m - 10m

    carb.log_info("  ✓ Stereo cameras configured")

    # Create ROS 2 Bridge action graph for stereo cameras
    stereo_graph = create_stereo_camera_graph(left_camera_prim_path, right_camera_prim_path)

    carb.log_info("=" * 80)
    carb.log_info("LUKEBOT vSLAM SETUP COMPLETE!")
    carb.log_info("=" * 80)
    carb.log_info(f"Robot location: {robot_prim_path}")
    carb.log_info("")
    carb.log_info("Stereo Camera Configuration:")
    carb.log_info(f"  - Model: OAK-D IOT 75 (stereo)")
    carb.log_info(f"  - Resolution: {resolution[0]}x{resolution[1]} per camera")
    carb.log_info(f"  - Horizontal FOV: {horizontal_fov_deg}°")
    carb.log_info(f"  - Baseline: {baseline * 1000:.1f}mm ({baseline}m)")
    carb.log_info(f"  - Depth Range: 0.35m - 10m")
    carb.log_info(f"  - Frame Rate: {frequency} FPS")
    carb.log_info("")
    carb.log_info("ROS 2 Topics Published:")
    carb.log_info("  - /stereo_camera/left/image_raw (sensor_msgs/Image)")
    carb.log_info("  - /stereo_camera/left/camera_info (sensor_msgs/CameraInfo)")
    carb.log_info("  - /stereo_camera/right/image_raw (sensor_msgs/Image)")
    carb.log_info("  - /stereo_camera/right/camera_info (sensor_msgs/CameraInfo)")
    carb.log_info("")
    carb.log_info("Next Steps to Run Isaac ROS vSLAM:")
    carb.log_info("  1. Start Isaac Sim (this script)")
    carb.log_info("  2. In separate terminal, launch Isaac ROS vSLAM:")
    carb.log_info("     ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py")
    carb.log_info("  3. Verify topics:")
    carb.log_info("     ros2 topic list")
    carb.log_info("     ros2 topic hz /stereo_camera/left/image_raw")
    carb.log_info("  4. Visualize in RViz2:")
    carb.log_info("     rviz2")
    carb.log_info("")
    carb.log_info("Robot will move forward slowly to generate SLAM data")
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

            # Move forward slowly to generate SLAM data
            # Vary movement slightly for interesting odometry
            if i < 500:
                # Forward
                my_lukebot.apply_wheel_actions(my_controller.forward(command=[0.15, 0.0, 0.0]))
            elif i < 700:
                # Turn slightly right while moving
                my_lukebot.apply_wheel_actions(my_controller.forward(command=[0.15, 0.0, -0.1]))
            elif i < 900:
                # Forward
                my_lukebot.apply_wheel_actions(my_controller.forward(command=[0.15, 0.0, 0.0]))
            elif i < 1100:
                # Turn slightly left while moving
                my_lukebot.apply_wheel_actions(my_controller.forward(command=[0.15, 0.0, 0.1]))
            else:
                # Continue forward
                my_lukebot.apply_wheel_actions(my_controller.forward(command=[0.15, 0.0, 0.0]))

            # Log camera data periodically
            if i % 120 == 0 and i > 0:
                try:
                    # Get images from both cameras
                    left_rgb = left_camera.get_rgb()
                    right_rgb = right_camera.get_rgb()

                    if left_rgb is not None and right_rgb is not None:
                        carb.log_info(f"[Frame {i}] Stereo cameras active")
                        carb.log_info(f"  Left: {left_rgb.shape}, Right: {right_rgb.shape}")

                        # Check for yellow target in left camera
                        if left_rgb.size > 0:
                            yellow_mask = (left_rgb[:,:,0] > 200) & (left_rgb[:,:,1] > 200) & (left_rgb[:,:,2] < 100)
                            yellow_pixels = np.sum(yellow_mask)
                            if yellow_pixels > 100:
                                carb.log_info(f"  → Yellow target visible ({yellow_pixels} pixels)")

                except Exception as e:
                    carb.log_warn(f"Camera data error: {e}")

            i += 1

    simulation_app.close()


if __name__ == "__main__":
    main()
