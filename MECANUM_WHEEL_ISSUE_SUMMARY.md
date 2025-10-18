# Lukebot Mecanum Wheel Issue - Root Cause Analysis

## Summary
The Lukebot mecanum wheel robot cannot perform holonomic movement (strafing, rotation) correctly. After extensive debugging, we've identified the root cause.

## Root Cause: WheeledRobot.apply_wheel_actions() Bug

### Evidence from Debug Tests

**Test:** Command individual wheel velocities `[5.0, 0, 0, 0]` (only front-left wheel)

**Expected Result:** Only front-left wheel spins at 5.0 rad/s

**Actual Result:** ALL wheels spin at ~0.7 rad/s (from logs)

```
Command: [5.0, 0.0, 0.0, 0.0]
Actual:  [0.71, 0.72, 0.72, 0.72]  # ALL wheels moving!
```

**Test 2:** Command `[0.0, 5.0, 0.0, 0.0]` (only front-right wheel)

**Actual Result:** ALL wheels spin at ~0.94 rad/s

```
Command: [0.0, 5.0, 0.0, 0.0]
Actual:  [0.94, 0.93, 0.94, 0.94]  # ALL wheels moving again!
```

### Conclusion

The `WheeledRobot.apply_wheel_actions()` method is broadcasting velocity commands to all wheels instead of applying them individually. This makes holonomic control impossible.

## What We Verified is CORRECT

1. ✅ **URDF is correct** - All 4 wheel joints properly defined
2. ✅ **Joint names are correct** - All found at expected paths
3. ✅ **DOF configuration is correct** - Articulation has 4 DOFs with correct names
4. ✅ **Wheel indices are correct** - `[0, 1, 2, 3]`
5. ✅ **Mecanum angles** - We tested multiple configurations

## The Initial Misdiagnosis

**What we thought:** Mecanum angles were wrong due to wheel axis direction mismatch (Kaya uses `axis="0 -1 0"`, Lukebot uses `axis="0 1 0"`)

**Reality:** The mecanum angles don't matter because the controller can't command individual wheels correctly in the first place!

## Why the Workaround in teleop.py Existed

The original code had this workaround:
```python
command_fixed = [command[1], command[0], -command[2]]  # Swap axes
```

This was attempting to compensate for the broken wheel control, but it couldn't fix the fundamental issue that all wheels move together.

## Solutions

### Option 1: Fix WheeledRobot (Not Recommended)
This would require modifying Isaac Sim core code, which we can't do.

### Option 2: Use Robot/Articulation Class Directly
Bypass `WheeledRobot` entirely and:
- Use `Robot` class from `isaacsim.core.api.robots`
- Implement custom mecanum kinematics
- Directly call `set_joint_velocity_targets()`

### Option 3: Manual Wheel Control
Calculate wheel velocities manually based on desired robot motion and apply them directly.

## Recommended Next Steps

1. **Implement manual mecanum kinematics** for Lukebot
2. **Use `Robot` class** instead of `WheeledRobot`
3. **Test with direct velocity commands** to verify individual wheel control works
4. **Report bug** to NVIDIA Isaac Sim team about `WheeledRobot.apply_wheel_actions()`

## Files for Reference

- Debug test: `lukebot_debug_wheels.py`
- Log file: Most recent in `C:/packman-repo/chk/kit-kernel/.../logs/Kit/Isaac-Sim Python/5.1/`
- URDF: `source/extensions/isaacsim.asset.importer.urdf/data/urdf/robots/lukebot/urdf/lukebot.urdf`

## Key Log Evidence

```
2025-10-18T20:46:27Z - ARTICULATION DOF INFORMATION:
  Number of DOFs: 4
  DOF names: ['front_left_wheel_joint', 'front_right_wheel_joint', 'rear_left_wheel_joint', 'rear_right_wheel_joint']
  Wheel indices array: [0, 1, 2, 3]  ← CORRECT

2025-10-18T20:46:28Z - >>> TEST: Index 0 only
  Command velocities: [5.0, 0.0, 0.0, 0.0]  ← Only wheel 0 should spin
  Actual DOF velocities: [0.71, 0.72, 0.72, 0.72]  ← ALL WHEELS SPINNING!
```

This proves the bug is in `WheeledRobot.apply_wheel_actions()`.
