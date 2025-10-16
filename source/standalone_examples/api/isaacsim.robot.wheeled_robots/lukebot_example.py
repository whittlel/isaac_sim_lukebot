# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""
Lukebot Mecanum Wheel Robot Example
This script creates a lukebot robot with:
- Mecanum wheel chassis
- OAK-D IOT 75 depth camera sensor
- Test scene for robot navigation
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import carb
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.controllers.base_controller import BaseController
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.stage import get_current_stage
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.robot.wheeled_robots.controllers.holonomic_controller import HolonomicController
from isaacsim.robot.wheeled_robots.robots.holonomic_robot_usd_setup import HolonomicRobotUsdSetup
from isaacsim.robot.wheeled_robots.robots.wheeled_robot import WheeledRobot
from isaacsim.sensors.camera import Camera
from omni.isaac.sensor import IMUSensor
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics

# Enable necessary extensions
enable_extension("omni.isaac.sensor")
enable_extension("isaacsim.asset.importer.urdf")


class LukebotController(BaseController):
    """
    Simple controller for holonomic mecanum wheel robot.
    Converts linear and angular velocity commands to wheel velocities.
    """

    def __init__(self, holonomic_controller: HolonomicController):
        super().__init__(name="lukebot_controller")
        self._holonomic_controller = holonomic_controller

    def forward(self, command):
        """
        command: [linear_velocity_x, linear_velocity_y, angular_velocity_z]
        """
        return self._holonomic_controller.forward(command=command)


def import_lukebot_urdf(prim_path: str, urdf_path: str):
    """Import URDF and set up mecanum wheel properties"""
    import omni.kit.commands

    # Create import configuration
    status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    import_config.merge_fixed_joints = False
    import_config.convex_decomp = False
    import_config.import_inertia_tensor = True
    import_config.fix_base = False
    import_config.make_default_prim = False
    import_config.self_collision = False
    import_config.create_physics_scene = False  # We'll create our own
    import_config.distance_scale = 1.0
    import_config.density = 0.0

    # Import URDF
    omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=urdf_path,
        import_config=import_config,
        dest_path=prim_path
    )

    stage = get_current_stage()
    robot_prim = stage.GetPrimAtPath(prim_path)

    if not robot_prim.IsValid():
        carb.log_error(f"Robot prim at {prim_path} is not valid")
        return None

    # Add mecanum wheel attributes to wheel joints
    wheel_joints = [
        f"{prim_path}/front_left_wheel_joint",
        f"{prim_path}/front_right_wheel_joint",
        f"{prim_path}/rear_left_wheel_joint",
        f"{prim_path}/rear_right_wheel_joint",
    ]

    # Mecanum wheel radius (50mm = 0.05m)
    wheel_radius = 0.050

    # Mecanum angles: +45 for left wheels, -45 for right wheels (in radians)
    # Front left: +45, Front right: -45, Rear left: -45, Rear right: +45
    mecanum_angles = [np.pi / 4, -np.pi / 4, -np.pi / 4, np.pi / 4]

    for joint_path, angle in zip(wheel_joints, mecanum_angles):
        joint_prim = stage.GetPrimAtPath(joint_path)
        if joint_prim.IsValid():
            # Add custom mecanum wheel attributes
            if not joint_prim.HasAttribute("isaacmecanumwheel:radius"):
                joint_prim.CreateAttribute("isaacmecanumwheel:radius", Sdf.ValueTypeNames.Float).Set(
                    wheel_radius
                )
            else:
                joint_prim.GetAttribute("isaacmecanumwheel:radius").Set(wheel_radius)

            if not joint_prim.HasAttribute("isaacmecanumwheel:angle"):
                joint_prim.CreateAttribute("isaacmecanumwheel:angle", Sdf.ValueTypeNames.Float).Set(angle)
            else:
                joint_prim.GetAttribute("isaacmecanumwheel:angle").Set(angle)

            # Set drive properties for the wheel joints
            drive_api = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
            drive_api.GetDampingAttr().Set(100.0)
            drive_api.GetStiffnessAttr().Set(0.0)

    return robot_prim


def setup_oak_d_camera(camera_prim_path: str):
    """
    Configure depth camera to mimic OAK-D IOT 75 specifications:
    - FOV: 75 degrees horizontal
    - Resolution: 1280x800 (native)
    - Depth range: 0.35m to 10m (typical)
    - Baseline: 7.5cm for stereo depth
    """
    stage = get_current_stage()

    # Create camera with Isaac Sim Camera class
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
        # Focal length calculation: focal_length = sensor_width / (2 * tan(hfov/2))
        horizontal_fov_deg = 75.0
        horizontal_aperture = 20.955  # mm (standard)
        focal_length = horizontal_aperture / (2 * np.tan(np.radians(horizontal_fov_deg) / 2))

        camera_api.GetFocalLengthAttr().Set(focal_length)
        camera_api.GetHorizontalApertureAttr().Set(horizontal_aperture)

        # Set clipping range (depth range)
        camera_api.GetClippingRangeAttr().Set(Gf.Vec2f(0.35, 10.0))

    return camera


def create_test_scene(world: World):
    """Create a test environment with obstacles and navigation challenges"""
    stage = get_current_stage()
    assets_root_path = get_assets_root_path()

    # Add various objects for testing depth perception and navigation
    # Box obstacle 1
    box1 = stage.DefinePrim("/World/Obstacles/Box1", "Cube")
    UsdGeom.Xform(box1).AddTranslateOp().Set(Gf.Vec3f(2.0, 1.0, 0.25))
    UsdGeom.Xform(box1).AddScaleOp().Set(Gf.Vec3f(0.5, 0.5, 0.5))
    box1.GetAttribute("primvars:displayColor").Set([(0.8, 0.2, 0.2)])
    UsdPhysics.CollisionAPI.Apply(box1)
    UsdPhysics.RigidBodyAPI.Apply(box1)
    physx_rigid_body_api = PhysxSchema.PhysxRigidBodyAPI.Apply(box1)
    physx_rigid_body_api.CreateDisableGravityAttr().Set(False)

    # Box obstacle 2
    box2 = stage.DefinePrim("/World/Obstacles/Box2", "Cube")
    UsdGeom.Xform(box2).AddTranslateOp().Set(Gf.Vec3f(-1.5, -1.5, 0.25))
    UsdGeom.Xform(box2).AddScaleOp().Set(Gf.Vec3f(0.3, 0.3, 0.5))
    box2.GetAttribute("primvars:displayColor").Set([(0.2, 0.8, 0.2)])
    UsdPhysics.CollisionAPI.Apply(box2)
    UsdPhysics.RigidBodyAPI.Apply(box2)
    physx_rigid_body_api = PhysxSchema.PhysxRigidBodyAPI.Apply(box2)
    physx_rigid_body_api.CreateDisableGravityAttr().Set(False)

    # Cylinder obstacle
    cylinder = stage.DefinePrim("/World/Obstacles/Cylinder1", "Cylinder")
    UsdGeom.Xform(cylinder).AddTranslateOp().Set(Gf.Vec3f(1.0, -2.0, 0.4))
    UsdGeom.Xform(cylinder).AddScaleOp().Set(Gf.Vec3f(0.2, 0.2, 0.4))
    cylinder.GetAttribute("primvars:displayColor").Set([(0.2, 0.2, 0.8)])
    UsdPhysics.CollisionAPI.Apply(cylinder)
    UsdPhysics.RigidBodyAPI.Apply(cylinder)
    physx_rigid_body_api = PhysxSchema.PhysxRigidBodyAPI.Apply(cylinder)
    physx_rigid_body_api.CreateDisableGravityAttr().Set(False)

    # Wall obstacles
    wall1 = stage.DefinePrim("/World/Obstacles/Wall1", "Cube")
    UsdGeom.Xform(wall1).AddTranslateOp().Set(Gf.Vec3f(0.0, 3.0, 0.5))
    UsdGeom.Xform(wall1).AddScaleOp().Set(Gf.Vec3f(4.0, 0.1, 1.0))
    wall1.GetAttribute("primvars:displayColor").Set([(0.7, 0.7, 0.7)])
    UsdPhysics.CollisionAPI.Apply(wall1)
    UsdPhysics.RigidBodyAPI.Apply(wall1)
    physx_rigid_body_api = PhysxSchema.PhysxRigidBodyAPI.Apply(wall1)
    physx_rigid_body_api.CreateDisableGravityAttr().Set(False)

    # Add a ramp for testing
    ramp = stage.DefinePrim("/World/Obstacles/Ramp", "Cube")
    UsdGeom.Xform(ramp).AddTranslateOp().Set(Gf.Vec3f(-2.0, 0.5, 0.1))
    UsdGeom.Xform(ramp).AddRotateXYZOp().Set(Gf.Vec3f(0, 0, 15))
    UsdGeom.Xform(ramp).AddScaleOp().Set(Gf.Vec3f(1.0, 0.5, 0.05))
    ramp.GetAttribute("primvars:displayColor").Set([(0.6, 0.6, 0.2)])
    UsdPhysics.CollisionAPI.Apply(ramp)
    UsdPhysics.RigidBodyAPI.Apply(ramp)
    physx_rigid_body_api = PhysxSchema.PhysxRigidBodyAPI.Apply(ramp)
    physx_rigid_body_api.CreateDisableGravityAttr().Set(False)


def main():
    # Create world
    my_world = World(stage_units_in_meters=1.0)
    my_world.scene.add_default_ground_plane()

    # Set camera view
    set_camera_view(eye=[4.0, 4.0, 3.0], target=[0.0, 0.0, 0.5], camera_prim_path="/OmniverseKit_Persp")

    # Get URDF path
    urdf_path = "C:/Users/Luke/Desktop/issac_sim/source/extensions/isaacsim.asset.importer.urdf/data/urdf/robots/lukebot/urdf/lukebot.urdf"
    robot_prim_path = "/World/Lukebot"

    # Import and setup robot
    carb.log_info("Importing Lukebot URDF...")
    robot_prim = import_lukebot_urdf(robot_prim_path, urdf_path)

    if robot_prim is None:
        carb.log_error("Failed to create Lukebot robot")
        simulation_app.close()
        return

    # Create wheeled robot object
    lukebot = WheeledRobot(
        prim_path=robot_prim_path,
        name="lukebot",
        wheel_dof_names=[
            "front_left_wheel_joint",
            "front_right_wheel_joint",
            "rear_left_wheel_joint",
            "rear_right_wheel_joint",
        ],
        create_robot=False,
    )

    # Setup holonomic controller
    holonomic_setup = HolonomicRobotUsdSetup(
        robot_prim_path=robot_prim_path, com_prim_path=f"{robot_prim_path}/chassis_link"
    )

    holonomic_controller = HolonomicController(
        name="holonomic_controller",
        wheel_radius=holonomic_setup.wheel_radius,
        wheel_positions=holonomic_setup.wheel_positions,
        wheel_orientations=holonomic_setup.wheel_orientations,
        mecanum_angles=holonomic_setup.mecanum_angles,
        wheel_axis=holonomic_setup.wheel_axis,
        up_axis=holonomic_setup.up_axis,
    )

    lukebot_controller = LukebotController(holonomic_controller=holonomic_controller)

    # Setup OAK-D camera
    camera_prim_path = f"{robot_prim_path}/oak_d_camera"
    carb.log_info("Setting up OAK-D IOT 75 camera...")
    camera = setup_oak_d_camera(camera_prim_path)

    # Add IMU sensor
    imu_prim_path = f"{robot_prim_path}/imu_link"
    imu_sensor = IMUSensor(prim_path=imu_prim_path, name="lukebot_imu", frequency=100, translation=np.array([0, 0, 0]))

    # Create test scene
    carb.log_info("Creating test scene...")
    create_test_scene(my_world)

    # Set initial robot position
    lukebot.set_world_pose(position=np.array([0.0, 0.0, 0.1]), orientation=euler_angles_to_quat(np.array([0, 0, 0])))

    # Reset world
    my_world.reset()
    lukebot.initialize()
    imu_sensor.initialize()

    carb.log_info("Lukebot initialized successfully!")
    carb.log_info("Camera resolution: 1280x800, FOV: 75 degrees")
    carb.log_info("Mecanum wheels configured for holonomic motion")

    # Test motion commands
    # Commands: [linear_x, linear_y, angular_z]
    test_commands = [
        np.array([0.0, 0.0, 0.0]),  # Stop
        np.array([0.5, 0.0, 0.0]),  # Forward
        np.array([0.0, 0.5, 0.0]),  # Strafe left
        np.array([0.0, 0.0, 0.5]),  # Rotate
        np.array([0.3, 0.3, 0.2]),  # Combined motion
    ]

    command_index = 0
    step_count = 0
    steps_per_command = 200

    carb.log_info("Starting simulation loop...")
    carb.log_info("Testing different motion commands:")
    carb.log_info("1. Stop (0 steps)")
    carb.log_info("2. Forward motion")
    carb.log_info("3. Strafe left")
    carb.log_info("4. Rotate in place")
    carb.log_info("5. Combined motion")

    while simulation_app.is_running():
        my_world.step(render=True)

        if my_world.is_playing():
            if step_count % steps_per_command == 0 and command_index < len(test_commands):
                current_command = test_commands[command_index]
                carb.log_info(
                    f"Command {command_index + 1}: linear=[{current_command[0]:.2f}, {current_command[1]:.2f}], angular={current_command[2]:.2f}"
                )
                command_index += 1

            if command_index < len(test_commands):
                # Apply motion command
                wheel_velocities = lukebot_controller.forward(command=test_commands[command_index])
                lukebot.apply_wheel_actions(
                    control_actions=lukebot.ArticulationAction(joint_velocities=wheel_velocities)
                )

                # Read and log IMU data every 50 steps
                if step_count % 50 == 0:
                    imu_data = imu_sensor.get_current_frame()
                    if imu_data is not None and "lin_acc_x" in imu_data:
                        carb.log_info(
                            f"IMU - Linear acc: [{imu_data['lin_acc_x']:.3f}, {imu_data['lin_acc_y']:.3f}, {imu_data['lin_acc_z']:.3f}]"
                        )

                # Get camera data periodically
                if step_count % 100 == 0:
                    camera_data = camera.get_current_frame()
                    if camera_data is not None:
                        carb.log_info(f"Camera data available: {camera_data.keys()}")

            step_count += 1

            # Reset after completing all commands
            if command_index >= len(test_commands) and step_count > len(test_commands) * steps_per_command + 100:
                carb.log_info("Test complete! Resetting...")
                my_world.reset()
                command_index = 0
                step_count = 0
                lukebot.set_world_pose(
                    position=np.array([0.0, 0.0, 0.1]), orientation=euler_angles_to_quat(np.array([0, 0, 0]))
                )

    simulation_app.close()


if __name__ == "__main__":
    main()
