"""The bridge's control logic without ROS: command arbitration, trajectory
playback against sim time, the poll loop's tick, and rate statistics.

`bridge_node.py` is a thin rclpy shell around this, so everything here is
exercised by plain pytest with the fake backend.
"""

from __future__ import annotations

import collections
import concurrent.futures
import math
import threading
import time
from typing import Callable, Deque, Dict, List, Optional

from . import so101
from .robot_api import Backend, RobotState, parse_state
from .trajectory import Trajectory, from_lists


class RateMeter:
    """Events per second and latency percentiles over a sliding window."""

    def __init__(self, window: float = 5.0):
        self.window = window
        self._stamps: Deque[float] = collections.deque()
        self._lat: Deque[tuple] = collections.deque()
        self._lock = threading.Lock()
        self.count = 0

    def tick(self, latency: Optional[float] = None, now: Optional[float] = None) -> None:
        now = time.monotonic() if now is None else now
        with self._lock:
            self.count += 1
            self._stamps.append(now)
            if latency is not None:
                self._lat.append((now, latency))
            self._trim(now)

    def _trim(self, now: float) -> None:
        while self._stamps and now - self._stamps[0] > self.window:
            self._stamps.popleft()
        while self._lat and now - self._lat[0][0] > self.window:
            self._lat.popleft()

    def rate(self, now: Optional[float] = None) -> float:
        now = time.monotonic() if now is None else now
        with self._lock:
            self._trim(now)
            if len(self._stamps) < 2:
                return 0.0
            span = self._stamps[-1] - self._stamps[0]
            return (len(self._stamps) - 1) / span if span > 0 else 0.0

    def latency(self, q: float) -> float:
        with self._lock:
            values = sorted(v for _, v in self._lat)
        if not values:
            return float("nan")
        index = min(len(values) - 1, max(0, int(math.ceil(q * len(values))) - 1))
        return values[index]


class BridgeCore:
    """Owns the command targets and turns state reads into publish calls."""

    def __init__(self, backend: Backend, command_rate_hz: float = 50.0,
                 on_state: Optional[Callable[[RobotState], None]] = None,
                 log: Callable[[str], None] = print):
        self.backend = backend
        self.command_period = 1.0 / command_rate_hz if command_rate_hz > 0 else 0.0
        self.on_state = on_state
        self.log = log
        self._lock = threading.Lock()
        self.trajectory: Optional[Trajectory] = None
        self.targets: Dict[str, float] = {}
        self._dirty = False
        self._last_sent: Dict[str, float] = {}
        self._last_send_time = -1e9
        self.state: Optional[RobotState] = None
        self.state_rate = RateMeter()
        self.command_rate = RateMeter()
        self.errors = 0
        self.last_error = ""
        self.trajectory_done_callbacks: List[Callable[[bool], None]] = []
        # False when the command tool itself answers the new state.
        self.parallel_read = True
        self._state_lock = threading.Lock()
        self._published_once = False
        self._pool = concurrent.futures.ThreadPoolExecutor(max_workers=2, thread_name_prefix="openbeam-io")

    # -- inputs (called from ROS callbacks) ---------------------------------
    def set_positions(self, positions: Dict[str, float]) -> None:
        clean = {n: so101.clamp(n, v) for n, v in positions.items() if n in so101.BY_NAME}
        unknown = [n for n in positions if n not in so101.BY_NAME]
        if unknown:
            raise ValueError(f"unknown joints {unknown}; expected {so101.JOINT_NAMES}")
        with self._lock:
            # Direct commands pre-empt a running trajectory for the joints they name.
            if self.trajectory is not None and set(clean) & set(self.trajectory.joint_names):
                self._finish_trajectory(False)
            self.targets.update(clean)
            self._dirty = True

    def set_trajectory(self, joint_names, times, positions, velocities=None) -> Trajectory:
        unknown = [n for n in joint_names if n not in so101.BY_NAME]
        if unknown:
            raise ValueError(f"unknown joints {unknown}; expected {so101.JOINT_NAMES}")
        state = self.state
        if state is None:
            raise RuntimeError("no robot state yet; is the game connected?")
        current = dict(state.positions)
        with self._lock:
            current.update({n: v for n, v in self.targets.items()})
            trajectory = from_lists(joint_names, times, positions, state.sim_time, current, velocities)
            if self.trajectory is not None:
                self._finish_trajectory(False)
            self.trajectory = trajectory
            self._dirty = True
        return trajectory

    def cancel_trajectory(self) -> None:
        with self._lock:
            if self.trajectory is not None:
                self._finish_trajectory(False)

    def _finish_trajectory(self, success: bool) -> None:
        self.trajectory = None
        for callback in list(self.trajectory_done_callbacks):
            try:
                callback(success)
            except Exception as error:  # pragma: no cover - a callback's own bug
                self.log(f"trajectory callback failed: {error}")

    # -- the poll loop ------------------------------------------------------
    def desired(self, sim_time: float) -> Dict[str, float]:
        with self._lock:
            if self.trajectory is not None:
                sample = self.trajectory.sample(sim_time)
                self.targets.update({n: so101.clamp(n, v) for n, v in sample.items()})
                self._dirty = True
                if self.trajectory.done(sim_time):
                    self._finish_trajectory(True)
            return dict(self.targets)

    def tick(self) -> Optional[RobotState]:
        """One poll: read state (or command-and-read), publish via on_state."""
        try:
            state = self.state
            sim_time = state.sim_time if state is not None else 0.0
            desired = self.desired(sim_time)
            now = time.monotonic()
            send = False
            with self._lock:
                changed = any(abs(desired.get(n, 0.0) - self._last_sent.get(n, math.inf)) > 1e-6 for n in desired)
                if desired and (self._dirty or changed) and now - self._last_send_time >= self.command_period:
                    send = True
                    self._dirty = False
            new_state = None
            if send:
                # The game services every request queued before a frame in that
                # frame, but a JSON-RPC batch runs one call per frame; so the
                # command and the state read go out on two connections at once
                # and the tick costs one frame, not two.
                started = time.monotonic()
                pending = self._pool.submit(self.backend.command_positions, desired)
                read = self._pool.submit(self.backend.read_state) if self.parallel_read else None
                answer = pending.result()
                self.command_rate.tick(time.monotonic() - started)
                self._last_sent = dict(desired)
                self._last_send_time = now
                if read is not None:
                    new_state = read.result()
                elif isinstance(answer, dict):
                    try:  # a combined command+state tool answers the state itself
                        new_state = parse_state(answer, joint_map=getattr(self.backend, "joint_map", None),
                                                fixed=getattr(self.backend, "fixed", None))
                        new_state.received_monotonic = time.monotonic()
                        new_state.request_seconds = new_state.received_monotonic - started
                    except ValueError:
                        new_state = None
            if new_state is None:
                new_state = self.backend.read_state()
            self._accept(new_state)
            return new_state
        except Exception as error:
            self.errors += 1
            self.last_error = f"{type(error).__name__}: {error}"
            if self.errors <= 5 or self.errors % 100 == 0:
                self.log(f"tick failed ({self.errors}): {self.last_error}")
            return None

    def _accept(self, new_state: RobotState) -> bool:
        """Publish a state unless a newer one already went out (pipelined reads
        can answer out of order)."""
        with self._state_lock:
            current = self.state
            if current is not None and new_state.sim_time < current.sim_time:
                return False
            if current is not None and new_state.sim_time == current.sim_time and self._published_once:
                self.state = new_state  # same physics instant: keep, don't republish
                return False
            self.state = new_state
            self._published_once = True
        self.state_rate.tick(new_state.request_seconds)
        if self.on_state is not None:
            self.on_state(new_state)
        return True

    def read_only(self) -> Optional[RobotState]:
        """An extra state read for links with a long round trip (the cloud):
        several in flight keep /joint_states at the game's frame rate."""
        try:
            new_state = self.backend.read_state()
            self._accept(new_state)
            return new_state
        except Exception as error:
            self.errors += 1
            self.last_error = f"{type(error).__name__}: {error}"
            return None

    def tracking_error(self) -> Dict[str, float]:
        state = self.state
        if state is None:
            return {}
        with self._lock:
            return {n: state.positions.get(n, 0.0) - v for n, v in self.targets.items()}
