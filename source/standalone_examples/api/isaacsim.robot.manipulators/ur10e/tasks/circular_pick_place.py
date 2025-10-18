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
from typing import Optional

import isaacsim.core.api.tasks as tasks
import numpy as np
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.storage.native import get_assets_root_path


class CircularPickPlace(tasks.PickPlace):
    """
    A task that picks up a cube and places it at positions in a circle around the robot base.

    Args:
        name: Name of the task
        cube_initial_position: Initial position of the cube (defaults to center)
        cube_initial_orientation: Initial orientation of the cube
        circle_radius: Radius of the circle for drop-off positions (in meters)
        num_positions: Number of positions around the circle
        cube_size: Size of the cube to pick up
        offset: Offset for the cube placement
    """
    def __init__(
        self,
        name: str = "ur10e_circular_pick_place",
        cube_initial_position: Optional[np.ndarray] = None,
        cube_initial_orientation: Optional[np.ndarray] = None,
        circle_radius: float = 0.5,
        num_positions: int = 8,
        cube_size: Optional[np.ndarray] = np.array([0.0515, 0.0515, 0.0515]),
        offset: Optional[np.ndarray] = None,
    ) -> None:
        # Calculate the first target position in the circle
        self.circle_radius = circle_radius
        self.num_positions = num_positions
        self.current_position_index = 0

        # Calculate initial target position (first point in circle)
        angle = 0
        target_position = np.array([
            circle_radius * np.cos(angle),
            circle_radius * np.sin(angle),
            cube_size[2] / 2.0
        ])

        # Set default cube initial position if not provided
        # Place it at a position within the robot's reach (slightly offset from center)
        if cube_initial_position is None:
            cube_initial_position = np.array([0.3, 0.3, cube_size[2] / 2.0])

        tasks.PickPlace.__init__(
            self,
            name=name,
            cube_initial_position=cube_initial_position,
            cube_initial_orientation=cube_initial_orientation,
            target_position=target_position,
            cube_size=cube_size,
            offset=offset,
        )
        return

    def set_robot(self) -> SingleManipulator:
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            raise Exception("Could not find Isaac Sim assets folder")
        asset_path = (
            assets_root_path + "/Isaac/Samples/Rigging/Manipulator/configure_manipulator/ur10e/ur/ur_gripper.usd"
        )
        add_reference_to_stage(usd_path=asset_path, prim_path="/ur")
        # define the gripper
        gripper = ParallelGripper(
            # We chose the following values while inspecting the articulation
            end_effector_prim_path="/ur/ee_link/robotiq_arg2f_base_link",
            joint_prim_names=["finger_joint"],
            joint_opened_positions=np.array([0]),
            joint_closed_positions=np.array([40]),
            action_deltas=np.array([-40]),
            use_mimic_joints=True,
        )
        # define the manipulator
        manipulator = SingleManipulator(
            prim_path="/ur",
            name="ur10_robot",
            end_effector_prim_path="/ur/ee_link/robotiq_arg2f_base_link",
            gripper=gripper,
        )
        return manipulator

    def get_next_target_position(self) -> np.ndarray:
        """
        Calculate the next target position in the circle.

        Returns:
            The next target position as a numpy array [x, y, z]
        """
        self.current_position_index = (self.current_position_index + 1) % self.num_positions
        angle = (2 * np.pi * self.current_position_index) / self.num_positions

        target_position = np.array([
            self.circle_radius * np.cos(angle),
            self.circle_radius * np.sin(angle),
            self._cube_size[2] / 2.0
        ])

        return target_position

    def update_target_position(self):
        """
        Update the target position to the next position in the circle.

        This updates the internal target position that the cube should be placed at.
        """
        new_target = self.get_next_target_position()
        # Update the target position stored in the task
        self._target_position = new_target
