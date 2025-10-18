# Test script to debug wheel configuration
# We know W (forward=0.5, lateral=0, rotation=0) causes counterclockwise rotation
# This means the axes are mixed up

import numpy as np

print("Current configuration:")
print("Wheel order: front_left, front_right, rear_left, rear_right")
print("Mecanum angles: [π/4, -π/4, -π/4, π/4]")
print("")
print("Test result: W (command [0.5, 0, 0]) causes CCW rotation")
print("Expected: Forward movement")
print("")
print("Diagnosis: The forward axis and rotation axis are swapped!")
print("")
print("Possible fixes to try:")
print("1. Invert all mecanum angles: [-π/4, π/4, π/4, -π/4]")
print("2. Change wheel order to match controller expectation")
print("3. Swap command axes when calling controller.forward()")
print("")
print("Let's try option 1 first - invert all angles:")
angles_original = [np.pi/4, -np.pi/4, -np.pi/4, np.pi/4]
angles_inverted = [-a for a in angles_original]
print(f"Original: {[f'{a:.4f}' for a in angles_original]}")
print(f"Inverted: {[f'{a:.4f}' for a in angles_inverted]}")
