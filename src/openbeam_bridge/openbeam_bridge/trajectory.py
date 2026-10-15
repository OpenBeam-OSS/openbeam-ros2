"""Joint-trajectory sampling, ROS-free so it can be unit tested anywhere.

Semantics follow ros2_controllers' joint_trajectory_controller closely enough
for course code: a new trajectory replaces the active one; if its first point
is later than t=0 the arm moves from where it is now; points with velocities
are joined by cubic Hermite segments, points without by straight lines; the
last point is held.
"""

from __future__ import annotations

import bisect
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence


@dataclass
class TrajectoryPoint:
    time: float  # seconds from trajectory start
    positions: List[float]
    velocities: Optional[List[float]] = None


@dataclass
class Trajectory:
    joint_names: List[str]
    points: List[TrajectoryPoint]
    start_time: float  # in the clock the sampler is driven with (sim seconds)
    start_positions: Dict[str, float] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.joint_names:
            raise ValueError("trajectory has no joint names")
        if len(set(self.joint_names)) != len(self.joint_names):
            raise ValueError("duplicate joint names")
        if not self.points:
            raise ValueError("trajectory has no points")
        last = -1.0
        for point in self.points:
            if len(point.positions) != len(self.joint_names):
                raise ValueError("a point's positions do not match joint_names")
            if point.velocities and len(point.velocities) != len(self.joint_names):
                raise ValueError("a point's velocities do not match joint_names")
            if point.time < last or point.time < 0.0:
                raise ValueError("time_from_start must be non-negative and non-decreasing")
            last = point.time
        missing = [name for name in self.joint_names if name not in self.start_positions]
        if missing and self.points[0].time > 0.0:
            raise ValueError(f"no current position for {missing}")
        # Prepend the current state as t=0 when the first point is in the future.
        if self.points[0].time > 0.0:
            self.points = [TrajectoryPoint(0.0, [self.start_positions[n] for n in self.joint_names],
                                           [0.0] * len(self.joint_names))] + list(self.points)
        self._times = [point.time for point in self.points]

    @property
    def duration(self) -> float:
        return self.points[-1].time

    def done(self, now: float) -> bool:
        return now - self.start_time >= self.duration

    def sample(self, now: float) -> Dict[str, float]:
        t = now - self.start_time
        if t <= 0.0:
            return dict(zip(self.joint_names, self.points[0].positions))
        if t >= self.duration:
            return dict(zip(self.joint_names, self.points[-1].positions))
        i = bisect.bisect_right(self._times, t) - 1
        a, b = self.points[i], self.points[i + 1]
        span = b.time - a.time
        if span <= 0.0:
            return dict(zip(self.joint_names, b.positions))
        s = (t - a.time) / span
        out = {}
        for k, name in enumerate(self.joint_names):
            p0, p1 = a.positions[k], b.positions[k]
            if a.velocities is not None and b.velocities is not None:
                v0, v1 = a.velocities[k] * span, b.velocities[k] * span
                h00 = 2 * s ** 3 - 3 * s ** 2 + 1
                h10 = s ** 3 - 2 * s ** 2 + s
                h01 = -2 * s ** 3 + 3 * s ** 2
                h11 = s ** 3 - s ** 2
                out[name] = h00 * p0 + h10 * v0 + h01 * p1 + h11 * v1
            else:
                out[name] = p0 + s * (p1 - p0)
        return out


def from_lists(joint_names: Sequence[str], times: Sequence[float], positions: Sequence[Sequence[float]],
               start_time: float, current: Dict[str, float],
               velocities: Optional[Sequence[Optional[Sequence[float]]]] = None) -> Trajectory:
    points = []
    for index, (time, pos) in enumerate(zip(times, positions)):
        vel = None
        if velocities is not None and velocities[index]:
            vel = list(velocities[index])
        points.append(TrajectoryPoint(float(time), list(pos), vel))
    return Trajectory(list(joint_names), points, start_time, dict(current))
