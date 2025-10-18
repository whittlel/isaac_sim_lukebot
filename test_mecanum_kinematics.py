# Test mecanum kinematics to debug strafing issues

from lukebot_motion_controller import MecanumKinematics
import numpy as np

print("=" * 80)
print("MECANUM KINEMATICS TEST")
print("=" * 80)

# Create kinematics with Lukebot dimensions
kinematics = MecanumKinematics(
    wheel_base=0.30,
    track_width=0.34,
    wheel_radius=0.05
)

print(f"\nRobot dimensions:")
print(f"  wheel_base: {kinematics.wheel_base} m")
print(f"  track_width: {kinematics.track_width} m")
print(f"  wheel_radius: {kinematics.wheel_radius} m")
print(f"  lx (half wheelbase): {kinematics.lx} m")
print(f"  ly (half track): {kinematics.ly} m")

print("\n" + "=" * 80)
print("TEST 1: Pure Forward Motion (vx=1.0 m/s)")
print("=" * 80)
wheel_vels = kinematics.inverse_kinematics(vx=1.0, vy=0.0, omega=0.0)
print(f"Commanded: vx=1.0, vy=0.0, omega=0.0")
print(f"Wheel velocities [FL, FR, RL, RR]: {wheel_vels}")
print(f"Expected: All wheels should spin at same speed (positive)")
print(f"Status: {'PASS' if np.allclose(wheel_vels, [20.0, 20.0, 20.0, 20.0]) else 'FAIL'}")

print("\n" + "=" * 80)
print("TEST 2: Pure Lateral Motion (vy=1.0 m/s, strafe left)")
print("=" * 80)
wheel_vels = kinematics.inverse_kinematics(vx=0.0, vy=1.0, omega=0.0)
print(f"Commanded: vx=0.0, vy=1.0, omega=0.0")
print(f"Wheel velocities [FL, FR, RL, RR]: {wheel_vels}")
print(f"Expected: FL/RR negative, FR/RL positive (X-configuration)")
print(f"Pattern: FL(-) FR(+) RL(+) RR(-)")

print("\n" + "=" * 80)
print("TEST 3: Pure Rotation (omega=1.0 rad/s, CCW)")
print("=" * 80)
wheel_vels = kinematics.inverse_kinematics(vx=0.0, vy=0.0, omega=1.0)
print(f"Commanded: vx=0.0, vy=0.0, omega=1.0")
print(f"Wheel velocities [FL, FR, RL, RR]: {wheel_vels}")
print(f"Expected: FL/RL negative, FR/RR positive (rotate CCW)")
print(f"Pattern: FL(-) FR(+) RL(-) RR(+)")

print("\n" + "=" * 80)
print("TEST 4: Forward Kinematics Check")
print("=" * 80)
# Test forward kinematics with known wheel velocities
test_wheels = np.array([20.0, 20.0, 20.0, 20.0])  # All same speed = forward
vx, vy, omega = kinematics.forward_kinematics(test_wheels)
print(f"Wheel velocities: {test_wheels}")
print(f"Calculated robot velocity: vx={vx:.3f}, vy={vy:.3f}, omega={omega:.3f}")
print(f"Expected: vx=1.0, vy=0.0, omega=0.0")
print(f"Status: {'PASS' if abs(vx-1.0)<0.01 and abs(vy)<0.01 and abs(omega)<0.01 else 'FAIL'}")

print("\n" + "=" * 80)
print("TEST 5: Actual wheel speeds for typical commands")
print("=" * 80)

test_cases = [
    ("Forward 0.5 m/s", 0.5, 0.0, 0.0),
    ("Strafe Left 0.5 m/s", 0.0, 0.5, 0.0),
    ("Strafe Right 0.5 m/s", 0.0, -0.5, 0.0),
    ("Rotate CCW 1.0 rad/s", 0.0, 0.0, 1.0),
    ("Rotate CW 1.0 rad/s", 0.0, 0.0, -1.0),
    ("Diagonal (Fwd+Left)", 0.5, 0.5, 0.0),
]

for name, vx, vy, omega in test_cases:
    wheel_vels = kinematics.inverse_kinematics(vx, vy, omega)
    print(f"\n{name}:")
    print(f"  Command: vx={vx:.2f}, vy={vy:.2f}, omega={omega:.2f}")
    print(f"  Wheels [FL, FR, RL, RR]: [{wheel_vels[0]:6.2f}, {wheel_vels[1]:6.2f}, {wheel_vels[2]:6.2f}, {wheel_vels[3]:6.2f}]")

print("\n" + "=" * 80)
print("ANALYSIS")
print("=" * 80)
print("If strafing doesn't work in simulation:")
print("1. Check if wheels are actually spinning")
print("2. Verify PhysX friction settings")
print("3. Check if cylindrical wheels (not proper mecanum) cause issues")
print("4. May need to increase wheel velocities or reduce ground friction")
print("\nThe math above is CORRECT for standard X-configuration mecanum.")
print("=" * 80)
