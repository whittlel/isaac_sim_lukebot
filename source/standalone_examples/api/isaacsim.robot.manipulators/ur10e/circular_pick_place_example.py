# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Circular Pick and Place Example

This example demonstrates a robot picking up a cube and placing it at different
positions arranged in a circle around the robot's base. The robot continuously
picks up the cube from its current location and drops it at the next position
in the circle.

The RMPFlowController is used to generate smooth, collision-aware motion while
the PickPlaceController manages the pick and place state machine.
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import numpy as np
from controller.pick_place import PickPlaceController
from isaacsim.core.api import World
from tasks.circular_pick_place import CircularPickPlace


def get_random_spawn_position(min_radius=0.3, max_radius=0.55, cube_half_height=0.02575):
    """
    Generate a random spawn position for the cube within the robot's reach.

    Args:
        min_radius: Minimum distance from robot base (meters) - default 0.3m
        max_radius: Maximum distance from robot base (meters) - default 0.55m
        cube_half_height: Half the height of the cube (to place it on the ground)

    Returns:
        A numpy array [x, y, z] with the spawn position
    """
    # Random angle around the robot
    angle = np.random.uniform(0, 2 * np.pi)
    # Random radius within the reachable area
    radius = np.random.uniform(min_radius, max_radius)

    # Calculate position
    x = radius * np.cos(angle)
    y = radius * np.sin(angle)
    z = cube_half_height  # Place on ground

    return np.array([x, y, z])


# Create the world
my_world = World(stage_units_in_meters=1.0, physics_dt=1 / 200, rendering_dt=20 / 200)

# Set cube size
cube_size = np.array([0.0515, 0.0515, 0.0515])

# Generate random initial position for the cube within robot's reach
initial_cube_position = get_random_spawn_position(
    min_radius=0.3,
    max_radius=0.55,
    cube_half_height=cube_size[2] / 2.0
)

print(f"Spawning cube at position: {initial_cube_position}")

# Initialize the Circular Pick Place task
# The cube starts at a random position and will be placed at 8 positions around a 0.5m radius circle
my_task = CircularPickPlace(
    name="ur10e_circular_pick_place",
    cube_initial_position=initial_cube_position,
    circle_radius=0.5,  # 0.5 meter radius circle
    num_positions=8,    # 8 positions around the circle (45 degrees apart)
    cube_size=cube_size
)

my_world.add_task(my_task)
my_world.reset()

# Get task parameters
task_params = my_world.get_task("ur10e_circular_pick_place").get_params()
ur10e_name = task_params["robot_name"]["value"]
cube_name = task_params["cube_name"]["value"]

# Get the robot object
my_ur10e = my_world.scene.get_object(ur10e_name)

# Initialize the pick and place controller (uses RMPFlowController internally)
my_controller = PickPlaceController(
    name="circular_controller",
    robot_articulation=my_ur10e,
    gripper=my_ur10e.gripper
)

articulation_controller = my_ur10e.get_articulation_controller()

reset_needed = False
cycle_count = 0  # Track how many complete cycles we've done
just_completed = False  # Flag to track if we just completed a cycle

print("Starting circular pick and place example...")
print(f"The robot will pick and place the cube at {my_task.num_positions} positions around a circle")
print(f"Circle radius: {my_task.circle_radius}m")
print(f"Initial cube position: {initial_cube_position}")

# Get cube object for respawning
cube_object = None

while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            reset_needed = False
            my_controller.reset()
            just_completed = False

        if my_world.current_time_step_index == 0:
            my_controller.reset()
            just_completed = False
            # Get the cube object after world is initialized
            if cube_object is None:
                cube_object = my_world.scene.get_object(cube_name)

        # Only update target and reset if we completed the previous cycle
        if just_completed:
            cycle_count += 1
            position_in_cycle = (cycle_count - 1) % my_task.num_positions + 1

            # After completing a full circle, spawn cube at a new random position
            if cycle_count % my_task.num_positions == 0:
                new_spawn_position = get_random_spawn_position(
                    min_radius=0.3,
                    max_radius=0.55,
                    cube_half_height=cube_size[2] / 2.0
                )
                print(f"\nCompleted full circle! Respawning cube at new position: {new_spawn_position}")
                cube_object.set_world_pose(position=new_spawn_position)
                # Reset velocity
                cube_object.set_linear_velocity(np.array([0.0, 0.0, 0.0]))
                cube_object.set_angular_velocity(np.array([0.0, 0.0, 0.0]))

            # Update target position for the NEXT cycle
            my_task.update_target_position()
            print(f"Next target position: {position_in_cycle}/{my_task.num_positions}")

            # Reset the controller to start a new pick and place cycle
            my_controller.reset()
            just_completed = False

        observations = my_world.get_observations()

        # Forward the observation values to the controller to get the actions
        actions = my_controller.forward(
            picking_position=observations[cube_name]["position"],
            placing_position=observations[cube_name]["target_position"],
            current_joint_positions=observations[ur10e_name]["joint_positions"],
            # Offset to approach from above - may need tuning
            end_effector_offset=np.array([0, 0, 0.20]),
        )

        # Check if the pick and place cycle is complete
        if my_controller.is_done() and not just_completed:
            print(f"Completed pick and place cycle {cycle_count + 1}")
            just_completed = True

        articulation_controller.apply_action(actions)

    if my_world.is_stopped():
        reset_needed = True

simulation_app.close()
