"""The SO-101 joint table the bridge publishes and accepts.

Names, order and limits follow TheRobotStudio SO-ARM100
`Simulation/SO101/so101_new_calib.urdf` (Apache-2.0, commit
a758567c3978dfeefe282ede0500085a48fe8f78), which is also the LeRobot naming
(`shoulder_pan` ... `gripper`) that OpenBeam's SO-101 actor uses. "new calib": every joint's zero is the middle of its
range. ROS units are radians; LeRobot's dataset units are degrees for the arm
joints and 0..100 (closed..open) for the gripper.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Sequence


@dataclass(frozen=True)
class Joint:
    name: str
    lower: float  # rad
    upper: float  # rad
    max_velocity: float  # rad/s (URDF says 10; the STS3215 manages ~5.2 unloaded)
    max_effort: float  # N*m


JOINTS: List[Joint] = [
    Joint("shoulder_pan", -1.91986, 1.91986, 10.0, 10.0),
    Joint("shoulder_lift", -1.74533, 1.74533, 10.0, 10.0),
    Joint("elbow_flex", -1.69, 1.69, 10.0, 10.0),
    Joint("wrist_flex", -1.65806, 1.65806, 10.0, 10.0),
    Joint("wrist_roll", -2.74385, 2.84121, 10.0, 10.0),
    Joint("gripper", -0.174533, 1.74533, 10.0, 10.0),
]

JOINT_NAMES: List[str] = [joint.name for joint in JOINTS]
ARM_JOINT_NAMES: List[str] = JOINT_NAMES[:5]
GRIPPER_JOINT = "gripper"
BY_NAME: Dict[str, Joint] = {joint.name: joint for joint in JOINTS}

# Frames from the URDF (robot_state_publisher publishes them from /joint_states).
BASE_FRAME = "base_link"
TOOL_FRAME = "gripper_frame_link"
WORLD_FRAME = "world"


def clamp(name: str, value: float) -> float:
    joint = BY_NAME[name]
    if math.isnan(value):
        raise ValueError(f"{name}: NaN command")
    return min(max(value, joint.lower), joint.upper)


def positions_from_array(values: Sequence[float]) -> Dict[str, float]:
    """A Float64MultiArray on /joint_commands: 5 arm values, or 6 with the gripper."""
    if len(values) not in (5, 6):
        raise ValueError(f"/joint_commands needs 5 or 6 values in the order {JOINT_NAMES}, got {len(values)}")
    return {name: clamp(name, float(value)) for name, value in zip(JOINT_NAMES, values)}


def gripper_rad_from_lerobot(percent_open: float) -> float:
    """LeRobot gripper 0 (closed) .. 100 (open) -> URDF radians."""
    joint = BY_NAME[GRIPPER_JOINT]
    fraction = min(max(percent_open / 100.0, 0.0), 1.0)
    return joint.lower + fraction * (joint.upper - joint.lower)


def gripper_lerobot_from_rad(radians: float) -> float:
    joint = BY_NAME[GRIPPER_JOINT]
    return 100.0 * (radians - joint.lower) / (joint.upper - joint.lower)
