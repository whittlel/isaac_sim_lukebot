# Check wheel drive mode configuration

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import carb
import omni.kit.commands
from isaacsim.core.utils.stage import get_current_stage
from pxr import UsdPhysics

# Import URDF
urdf_path = "C:/Users/Luke/Desktop/issac_sim/source/extensions/isaacsim.asset.importer.urdf/data/urdf/robots/lukebot/urdf/lukebot.urdf"

status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
import_config.merge_fixed_joints = False
import_config.convex_decomp = False
import_config.import_inertia_tensor = True
import_config.fix_base = False

omni.kit.commands.execute("URDFParseAndImportFile", urdf_path=urdf_path, import_config=import_config)

stage = get_current_stage()

robot_prim_path = "/lukebot"
wheel_joints = [
    "front_left_wheel_joint",
    "front_right_wheel_joint",
    "rear_left_wheel_joint",
    "rear_right_wheel_joint"
]

carb.log_info("=" * 80)
carb.log_info("CHECKING DRIVE API CONFIGURATION")
carb.log_info("=" * 80)

for joint_name in wheel_joints:
    joint_path = f"{robot_prim_path}/joints/{joint_name}"
    joint_prim = stage.GetPrimAtPath(joint_path)

    if joint_prim.IsValid():
        carb.log_info(f"\n{joint_name}:")
        carb.log_info(f"  Path: {joint_path}")

        # Check if DriveAPI is applied
        if joint_prim.HasAPI(UsdPhysics.DriveAPI, "angular"):
            drive = UsdPhysics.DriveAPI.Get(joint_prim, "angular")
            carb.log_info(f"  ✓ DriveAPI(angular) exists")
            carb.log_info(f"    - Type: {drive.GetTypeAttr().Get()}")
            carb.log_info(f"    - Damping: {drive.GetDampingAttr().Get()}")
            carb.log_info(f"    - Stiffness: {drive.GetStiffnessAttr().Get()}")
            carb.log_info(f"    - MaxForce: {drive.GetMaxForceAttr().Get()}")
        else:
            carb.log_warn(f"  ✗ NO DriveAPI(angular)!")
    else:
        carb.log_error(f"  ✗ Joint not found: {joint_path}")

carb.log_info("\n" + "=" * 80)

simulation_app.close()
