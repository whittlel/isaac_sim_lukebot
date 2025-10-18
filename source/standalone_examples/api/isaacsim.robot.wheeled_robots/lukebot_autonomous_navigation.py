# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Lukebot Autonomous Navigation - Object Finding Demo
Features:
- Detailed test environment with multiple rooms and obstacles
- Overhead camera for monitoring
- Autonomous object detection and navigation to yellow target
- ROS 2 Bridge for vSLAM integration
- State machine for search behavior
- Uses new motion controller with physics bypass for reliable movement
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
import omni.graph.core as og
import usdrt.Sdf

# Import our motion controller
from lukebot_motion_controller import create_motion_controller

# Enable necessary extensions
enable_extension("omni.isaac.sensor")

# Try to enable ROS 2 Bridge
ROS2_AVAILABLE = False
try:
    enable_extension("isaacsim.ros2.bridge")
    simulation_app.update()
    import omni.kit.app
    extension_manager = omni.kit.app.get_app().get_extension_manager()
    if extension_manager.is_extension_enabled("isaacsim.ros2.bridge"):
        ROS2_AVAILABLE = True
        carb.log_info("ROS 2 Bridge extension enabled successfully")
    else:
        carb.log_warn("ROS 2 Bridge extension failed to start")
except Exception as e:
    carb.log_warn(f"ROS 2 Bridge not available: {e}")


def create_stereo_camera_graph(left_camera_path, right_camera_path):
    """Create ROS 2 action graphs for stereo camera publishing"""
    try:
        carb.log_info("Creating ROS 2 stereo camera action graphs...")
        keys = og.Controller.Keys

        # LEFT CAMERA GRAPH
        (left_graph, _, _, _) = og.Controller.edit(
            {
                "graph_path": "/World/StereoCamera_Left_Graph",
                "evaluator_name": "push",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
            },
            {
                keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("createViewport", "isaacsim.core.nodes.IsaacCreateViewport"),
                    ("getRenderProduct", "isaacsim.core.nodes.IsaacGetViewportRenderProduct"),
                    ("setCamera", "isaacsim.core.nodes.IsaacSetCameraOnRenderProduct"),
                    ("cameraHelperRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ],
                keys.CONNECT: [
                    ("OnTick.outputs:tick", "createViewport.inputs:execIn"),
                    ("createViewport.outputs:execOut", "getRenderProduct.inputs:execIn"),
                    ("createViewport.outputs:viewport", "getRenderProduct.inputs:viewport"),
                    ("getRenderProduct.outputs:execOut", "setCamera.inputs:execIn"),
                    ("getRenderProduct.outputs:renderProductPath", "setCamera.inputs:renderProductPath"),
                    ("setCamera.outputs:execOut", "cameraHelperRgb.inputs:execIn"),
                    ("getRenderProduct.outputs:renderProductPath", "cameraHelperRgb.inputs:renderProductPath"),
                ],
                keys.SET_VALUES: [
                    ("createViewport.inputs:viewportId", 0),
                    ("cameraHelperRgb.inputs:frameId", "camera_left"),
                    ("cameraHelperRgb.inputs:topicName", "stereo_camera/left"),
                    ("cameraHelperRgb.inputs:type", "rgb"),
                    ("setCamera.inputs:cameraPrim", [usdrt.Sdf.Path(left_camera_path)]),
                ],
            },
        )
        og.Controller.evaluate_sync(left_graph)

        # RIGHT CAMERA GRAPH
        (right_graph, _, _, _) = og.Controller.edit(
            {
                "graph_path": "/World/StereoCamera_Right_Graph",
                "evaluator_name": "push",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
            },
            {
                keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("createViewport", "isaacsim.core.nodes.IsaacCreateViewport"),
                    ("getRenderProduct", "isaacsim.core.nodes.IsaacGetViewportRenderProduct"),
                    ("setCamera", "isaacsim.core.nodes.IsaacSetCameraOnRenderProduct"),
                    ("cameraHelperRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ],
                keys.CONNECT: [
                    ("OnTick.outputs:tick", "createViewport.inputs:execIn"),
                    ("createViewport.outputs:execOut", "getRenderProduct.inputs:execIn"),
                    ("createViewport.outputs:viewport", "getRenderProduct.inputs:viewport"),
                    ("getRenderProduct.outputs:execOut", "setCamera.inputs:execIn"),
                    ("getRenderProduct.outputs:renderProductPath", "setCamera.inputs:renderProductPath"),
                    ("setCamera.outputs:execOut", "cameraHelperRgb.inputs:execIn"),
                    ("getRenderProduct.outputs:renderProductPath", "cameraHelperRgb.inputs:renderProductPath"),
                ],
                keys.SET_VALUES: [
                    ("createViewport.inputs:viewportId", 1),
                    ("cameraHelperRgb.inputs:frameId", "camera_right"),
                    ("cameraHelperRgb.inputs:topicName", "stereo_camera/right"),
                    ("cameraHelperRgb.inputs:type", "rgb"),
                    ("setCamera.inputs:cameraPrim", [usdrt.Sdf.Path(right_camera_path)]),
                ],
            },
        )
        og.Controller.evaluate_sync(right_graph)

        carb.log_info("  ✓ ROS 2 stereo camera graphs created successfully")
        return (left_graph, right_graph)
    except Exception as e:
        carb.log_error(f"Failed to create stereo camera graphs: {e}")
        return None


def create_detailed_environment(stage):
    """Create a detailed test environment with rooms, walls, and objects"""
    carb.log_info("Creating detailed test environment...")

    # Room dimensions
    room_size = 8.0  # 8m x 8m room
    wall_height = 2.0
    wall_thickness = 0.2

    # Create outer walls
    walls = [
        # North wall
        ("/World/Environment/WallNorth", Gf.Vec3f(0.0, room_size/2, wall_height/2), Gf.Vec3f(room_size, wall_thickness, wall_height)),
        # South wall
        ("/World/Environment/WallSouth", Gf.Vec3f(0.0, -room_size/2, wall_height/2), Gf.Vec3f(room_size, wall_thickness, wall_height)),
        # East wall
        ("/World/Environment/WallEast", Gf.Vec3f(room_size/2, 0.0, wall_height/2), Gf.Vec3f(wall_thickness, room_size, wall_height)),
        # West wall
        ("/World/Environment/WallWest", Gf.Vec3f(-room_size/2, 0.0, wall_height/2), Gf.Vec3f(wall_thickness, room_size, wall_height)),
    ]

    for path, pos, scale in walls:
        wall = stage.DefinePrim(path, "Cube")
        UsdGeom.Xform(wall).AddTranslateOp().Set(pos)
        UsdGeom.Xform(wall).AddScaleOp().Set(scale)
        wall.GetAttribute("primvars:displayColor").Set([(0.8, 0.8, 0.8)])
        UsdPhysics.CollisionAPI.Apply(wall)
        # Make wall STATIC (no RigidBodyAPI = static collider)

    # Interior walls to create "rooms" (positioned away from robot spawn at origin)
    interior_walls = [
        # Vertical divider (moved to X=2.5m to avoid robot at origin)
        ("/World/Environment/DividerV1", Gf.Vec3f(2.5, 0.0, wall_height/2), Gf.Vec3f(wall_thickness, 3.0, wall_height)),
        # Horizontal partial wall
        ("/World/Environment/DividerH1", Gf.Vec3f(-2.0, 2.0, wall_height/2), Gf.Vec3f(2.5, wall_thickness, wall_height)),
    ]

    for path, pos, scale in interior_walls:
        wall = stage.DefinePrim(path, "Cube")
        UsdGeom.Xform(wall).AddTranslateOp().Set(pos)
        UsdGeom.Xform(wall).AddScaleOp().Set(scale)
        wall.GetAttribute("primvars:displayColor").Set([(0.7, 0.7, 0.7)])
        UsdPhysics.CollisionAPI.Apply(wall)
        # Make wall STATIC (no RigidBodyAPI = static collider)

    # TARGET OBJECT (Yellow Cube - what the robot searches for)
    target = stage.DefinePrim("/World/Environment/TargetCube", "Cube")
    UsdGeom.Xform(target).AddTranslateOp().Set(Gf.Vec3f(3.0, 2.5, 0.3))
    UsdGeom.Xform(target).AddScaleOp().Set(Gf.Vec3f(0.6, 0.6, 0.6))
    target.GetAttribute("primvars:displayColor").Set([(1.0, 1.0, 0.0)])  # Bright yellow
    UsdPhysics.CollisionAPI.Apply(target)
    # Target is STATIC so it stays in place

    # Obstacles (various colors and shapes)
    obstacles = [
        # Red boxes
        ("/World/Environment/ObstacleRed1", "Cube", Gf.Vec3f(2.0, -1.5, 0.25), Gf.Vec3f(0.5, 0.5, 0.5), (0.9, 0.2, 0.2)),
        ("/World/Environment/ObstacleRed2", "Cube", Gf.Vec3f(-2.5, -2.0, 0.3), Gf.Vec3f(0.6, 0.6, 0.6), (0.9, 0.2, 0.2)),
        # Green cylinders
        ("/World/Environment/ObstacleGreen1", "Cylinder", Gf.Vec3f(-1.5, 1.5, 0.4), Gf.Vec3f(0.2, 0.2, 0.4), (0.2, 0.9, 0.2)),
        ("/World/Environment/ObstacleGreen2", "Cylinder", Gf.Vec3f(1.5, 0.5, 0.3), Gf.Vec3f(0.15, 0.15, 0.3), (0.2, 0.9, 0.2)),
        # Blue boxes
        ("/World/Environment/ObstacleBlue1", "Cube", Gf.Vec3f(-3.0, -1.0, 0.2), Gf.Vec3f(0.4, 0.4, 0.4), (0.2, 0.2, 0.9)),
        ("/World/Environment/ObstacleBlue2", "Cube", Gf.Vec3f(2.5, 1.5, 0.25), Gf.Vec3f(0.5, 0.5, 0.5), (0.2, 0.2, 0.9)),
        # Purple cylinders
        ("/World/Environment/ObstaclePurple1", "Cylinder", Gf.Vec3f(0.5, -2.5, 0.35), Gf.Vec3f(0.18, 0.18, 0.35), (0.7, 0.2, 0.9)),
        ("/World/Environment/ObstaclePurple2", "Cylinder", Gf.Vec3f(-1.0, 0.0, 0.3), Gf.Vec3f(0.15, 0.15, 0.3), (0.7, 0.2, 0.9)),
    ]

    for path, prim_type, pos, scale, color in obstacles:
        obj = stage.DefinePrim(path, prim_type)
        UsdGeom.Xform(obj).AddTranslateOp().Set(pos)
        UsdGeom.Xform(obj).AddScaleOp().Set(scale)
        obj.GetAttribute("primvars:displayColor").Set([color])
        UsdPhysics.CollisionAPI.Apply(obj)
        # Obstacles are STATIC (no RigidBodyAPI = static collider)

    carb.log_info("  ✓ Created detailed environment with walls and obstacles")


class AutonomousNavigationBehavior:
    """State machine for autonomous object finding"""

    def __init__(self, motion_controller):
        self.motion_controller = motion_controller
        self.state = "SEARCHING"  # States: SEARCHING, APPROACHING, REACHED, CELEBRATING
        self.search_pattern_index = 0
        self.frames_in_state = 0
        self.target_visible = False
        self.target_pixel_count = 0
        self.last_seen_direction = 0.0

    def update(self, left_camera_rgb, robot):
        """Update behavior based on current state and camera input"""
        self.frames_in_state += 1

        # Detect yellow target in camera
        if left_camera_rgb is not None and left_camera_rgb.size > 0:
            yellow_mask = (left_camera_rgb[:,:,0] > 200) & (left_camera_rgb[:,:,1] > 200) & (left_camera_rgb[:,:,2] < 100)
            self.target_pixel_count = np.sum(yellow_mask)
            self.target_visible = self.target_pixel_count > 500

            if self.target_visible:
                # Calculate centroid to determine direction
                rows, cols = np.where(yellow_mask)
                if len(cols) > 0:
                    centroid_x = np.mean(cols)
                    image_center = left_camera_rgb.shape[1] / 2
                    # Positive = target on right, negative = target on left
                    self.last_seen_direction = (centroid_x - image_center) / image_center

        # State machine
        if self.state == "SEARCHING":
            return self._search_behavior()
        elif self.state == "APPROACHING":
            return self._approach_behavior()
        elif self.state == "REACHED":
            return self._reached_behavior()
        elif self.state == "CELEBRATING":
            return self._celebrate_behavior()

    def _search_behavior(self):
        """Search for the yellow target using a sweep pattern"""
        if self.target_visible:
            carb.log_info(f"TARGET FOUND! ({self.target_pixel_count} pixels) - Switching to APPROACHING")
            self.state = "APPROACHING"
            self.frames_in_state = 0
            return (0.0, 0.0, 0.0)  # Stop momentarily

        # Search pattern: rotate in place with occasional forward movement
        if self.frames_in_state < 60:
            # Rotate slowly
            return (0.0, 0.0, 0.3)
        elif self.frames_in_state < 80:
            # Move forward a bit
            return (0.2, 0.0, 0.0)
        else:
            # Reset pattern
            self.frames_in_state = 0
            return (0.0, 0.0, 0.3)

    def _approach_behavior(self):
        """Navigate toward the yellow target"""
        if not self.target_visible:
            carb.log_warn("Target lost! Returning to SEARCHING")
            self.state = "SEARCHING"
            self.frames_in_state = 0
            return (0.0, 0.0, 0.0)

        # Check if we've reached the target (large pixel count)
        if self.target_pixel_count > 150000:  # Target fills most of view
            carb.log_info("TARGET REACHED!")
            self.state = "REACHED"
            self.frames_in_state = 0
            return (0.0, 0.0, 0.0)

        # Navigate: adjust heading based on target position, move forward
        turn_speed = -self.last_seen_direction * 0.4  # Proportional control
        forward_speed = 0.25

        # Slow down as we get closer
        if self.target_pixel_count > 50000:
            forward_speed = 0.15

        return (forward_speed, 0.0, turn_speed)

    def _reached_behavior(self):
        """Stop at the target"""
        if self.frames_in_state < 60:
            return (0.0, 0.0, 0.0)
        else:
            carb.log_info("Mission complete! Starting celebration...")
            self.state = "CELEBRATING"
            self.frames_in_state = 0
            return (0.0, 0.0, 0.0)

    def _celebrate_behavior(self):
        """Victory spin!"""
        if self.frames_in_state < 180:
            return (0.0, 0.0, 0.5)
        else:
            # Reset to searching
            self.state = "SEARCHING"
            self.frames_in_state = 0
            return (0.0, 0.0, 0.0)


def main():
    # Create world
    my_world = World(stage_units_in_meters=1.0)
    my_world.scene.add_default_ground_plane()

    # Set OVERHEAD camera view
    set_camera_view(
        eye=[0.0, 0.0, 12.0],      # Directly above at 12m height
        target=[0.0, 0.0, 0.0],     # Looking at origin
        camera_prim_path="/OmniverseKit_Persp"
    )

    # Get URDF path
    urdf_path = "C:/Users/Luke/Desktop/issac_sim/source/extensions/isaacsim.asset.importer.urdf/data/urdf/robots/lukebot/urdf/lukebot.urdf"

    carb.log_info("=" * 80)
    carb.log_info("LUKEBOT AUTONOMOUS NAVIGATION - OBJECT FINDING DEMO")
    carb.log_info("=" * 80)
    carb.log_info("Importing Lukebot URDF...")

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

    # Find the imported robot
    robot_prim = None
    for prim in stage.Traverse():
        if prim.GetName() == "lukebot":
            robot_prim = prim
            robot_prim_path = str(prim.GetPath())
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

    # Create stereo cameras
    camera_center_position = np.array([0.235, 0.0, 0.09])
    baseline = 0.075
    half_baseline = baseline / 2.0

    left_camera_position = camera_center_position + np.array([0.0, half_baseline, 0.0])
    right_camera_position = camera_center_position + np.array([0.0, -half_baseline, 0.0])

    resolution = (1280, 800)
    frequency = 30
    horizontal_fov_deg = 75.0
    horizontal_aperture = 20.955
    focal_length = horizontal_aperture / (2 * np.tan(np.radians(horizontal_fov_deg) / 2))
    vertical_aperture = horizontal_aperture * (800.0 / 1280.0)

    left_camera_prim_path = "/World/Lukebot_Camera_Left"
    right_camera_prim_path = "/World/Lukebot_Camera_Right"

    left_camera = Camera(
        prim_path=left_camera_prim_path,
        position=left_camera_position,
        frequency=frequency,
        resolution=resolution,
        orientation=np.array([1.0, 0.0, 0.0, 0.0]),
    )

    right_camera = Camera(
        prim_path=right_camera_prim_path,
        position=right_camera_position,
        frequency=frequency,
        resolution=resolution,
        orientation=np.array([1.0, 0.0, 0.0, 0.0]),
    )

    # Create detailed test environment
    create_detailed_environment(stage)

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

    # Initialize autonomous behavior
    nav_behavior = AutonomousNavigationBehavior(motion_controller)

    # Reset world
    my_world.reset()

    # Initialize cameras
    left_camera.initialize()
    right_camera.initialize()
    left_camera.set_focal_length(focal_length)
    left_camera.set_horizontal_aperture(horizontal_aperture)
    left_camera.set_vertical_aperture(vertical_aperture)
    left_camera.set_clipping_range(0.35, 10.0)
    right_camera.set_focal_length(focal_length)
    right_camera.set_horizontal_aperture(horizontal_aperture)
    right_camera.set_vertical_aperture(vertical_aperture)
    right_camera.set_clipping_range(0.35, 10.0)

    # Create ROS 2 graphs if available
    if ROS2_AVAILABLE:
        create_stereo_camera_graph(left_camera_prim_path, right_camera_prim_path)

    carb.log_info("=" * 80)
    carb.log_info("AUTONOMOUS NAVIGATION DEMO READY!")
    carb.log_info("=" * 80)
    carb.log_info("Mission: Find and navigate to the YELLOW TARGET cube")
    carb.log_info("Behavior: Search → Approach → Reach → Celebrate → Repeat")
    carb.log_info("View: Overhead camera at 12m height")
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
                nav_behavior = AutonomousNavigationBehavior(motion_controller)
                reset_needed = False
                i = 0

            # Get camera data
            left_rgb = left_camera.get_rgb()

            # Update autonomous behavior
            vx, vy, omega = nav_behavior.update(left_rgb, my_lukebot)
            motion_controller.set_velocity(vx, vy, omega)
            motion_controller.update(dt=1.0/60.0)  # 60 FPS

            # Log status periodically
            if i % 120 == 0 and i > 0:
                carb.log_info(f"[Frame {i}] State: {nav_behavior.state}, Target pixels: {nav_behavior.target_pixel_count}")

            i += 1

    simulation_app.close()


if __name__ == "__main__":
    main()
