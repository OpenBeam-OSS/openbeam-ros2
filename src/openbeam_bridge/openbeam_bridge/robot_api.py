"""What the bridge needs from a simulator, and two ways to get it.

`McpBackend` drives an OpenBeam game (local or hosted) through its MCP tools.
`FakeBackend` is a kinematic SO-101 with first-order position servos, for the
unit tests and for trying the ROS side without a game.

Both answer the same `RobotState`, already in ROS conventions: radians,
metres, REP-103 axes (x forward, y left, z up) for the base pose.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

from . import so101
from .mcp_client import McpClient, McpError, ToolError


@dataclass
class RobotState:
    sim_time: float
    positions: Dict[str, float]
    velocities: Dict[str, float] = field(default_factory=dict)
    efforts: Dict[str, float] = field(default_factory=dict)
    # base_link in the world frame, ROS axes: (x, y, z), (qx, qy, qz, qw)
    base_position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    base_orientation: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    frame: Optional[int] = None
    extras: Dict[str, Any] = field(default_factory=dict)  # contacts, beam stress, ...
    received_monotonic: float = 0.0
    request_seconds: float = 0.0  # wall time the read took


@dataclass
class Image:
    mime: str
    data: bytes
    sim_time: Optional[float] = None
    request_seconds: float = 0.0
    fov_y_deg: Optional[float] = None
    # camera_link in the world, ROS axes (x forward, z up), when the game reports it
    camera_position: Optional[Tuple[float, float, float]] = None
    camera_orientation: Optional[Tuple[float, float, float, float]] = None


class Backend:
    name = "backend"

    def connect(self) -> Dict[str, Any]:
        return {}

    def read_state(self) -> RobotState:
        raise NotImplementedError

    def command_positions(self, positions: Dict[str, float]) -> Any:
        raise NotImplementedError

    def capture_image(self) -> Optional[Image]:
        return None

    camera_pose_live = False

    def close(self) -> None:
        pass


# ----------------------------------------------------------------- axes ----
# OpenBeam (Ogre) world: right-handed, +Y up, the camera's forward is -Z.
# ROS REP-103: x forward, y left, z up.  x_ros = -z_g, y_ros = -x_g, z_ros = y_g.
def game_to_ros_vector(v) -> Tuple[float, float, float]:
    x, y, z = float(v[0]), float(v[1]), float(v[2])
    return (-z, -x, y)


def game_to_ros_quaternion(q) -> Tuple[float, float, float, float]:
    """Quaternion (x, y, z, w) of a game-frame rotation, re-expressed in ROS axes.

    The axis change is a proper rotation C, so R_ros = C R_g C^T; for a
    quaternion that is the vector part mapped like a vector.
    """
    x, y, z, w = (float(c) for c in q)
    vx, vy, vz = game_to_ros_vector((x, y, z))
    return (vx, vy, vz, w)


# -------------------------------------------------------------- the game ---
DEFAULT_TOOLS = {
    # The game's robot tools; see docs/GAME_CONTRACT.md.  Override with ROS parameters.
    "state": "robot_state",
    "command": "robot_set_joints",
    "image": "screenshot",
}


def parse_joint_map(text: str) -> Dict[str, str]:
    """"base_pan:shoulder_pan,shoulder:shoulder_lift" -> {game name: ROS name}."""
    out: Dict[str, str] = {}
    for item in (text or "").split(","):
        item = item.strip()
        if not item:
            continue
        game, _, ros = item.partition(":")
        if not ros:
            raise ValueError(f"joint_map entry {item!r} is not game_name:ros_name")
        if ros.strip() not in so101.BY_NAME:
            raise ValueError(f"joint_map maps to {ros!r}, not an SO-101 joint")
        out[game.strip()] = ros.strip()
    return out


def parse_fixed(text: str) -> Dict[str, float]:
    """"wrist_flex=0,gripper=0.5" -> joints the game does not have, held at a value."""
    out: Dict[str, float] = {}
    for item in (text or "").split(","):
        item = item.strip()
        if item:
            name, _, value = item.partition("=")
            if name.strip() not in so101.BY_NAME:
                raise ValueError(f"fixed joint {name!r} is not an SO-101 joint")
            out[name.strip()] = float(value or 0.0)
    return out


def parse_state(payload: Dict[str, Any], names: List[str] = so101.JOINT_NAMES,
                joint_map: Optional[Dict[str, str]] = None, fixed: Optional[Dict[str, float]] = None) -> RobotState:
    """Accept the contract's shape, plus two common variants, so the bridge
    survives small naming changes on the game side."""
    if not isinstance(payload, dict):
        raise ValueError("state is not an object")
    body = payload.get("robot", payload) if isinstance(payload.get("robot"), dict) and "joints" not in payload else payload
    joints = body.get("joints")
    pos: Dict[str, float] = {}
    vel: Dict[str, float] = {}
    eff: Dict[str, float] = {}
    if isinstance(joints, list):
        for row in joints:
            if not isinstance(row, dict) or "name" not in row:
                continue
            name = row["name"]
            if "position" in row and row["position"] is not None:
                pos[name] = float(row["position"])
            if row.get("velocity") is not None:
                vel[name] = float(row["velocity"])
            if row.get("effort") is not None:
                eff[name] = float(row["effort"])
    elif isinstance(joints, dict):
        for name, row in joints.items():
            if isinstance(row, dict):
                if row.get("position") is not None:
                    pos[name] = float(row["position"])
                if row.get("velocity") is not None:
                    vel[name] = float(row["velocity"])
                if row.get("effort") is not None:
                    eff[name] = float(row["effort"])
            elif row is not None:
                pos[name] = float(row)
    elif isinstance(body.get("position"), list) and isinstance(body.get("name"), list):
        for i, name in enumerate(body["name"]):
            pos[name] = float(body["position"][i])
            if i < len(body.get("velocity") or []):
                vel[name] = float(body["velocity"][i])
            if i < len(body.get("effort") or []):
                eff[name] = float(body["effort"][i])
    if joint_map:
        pos = {joint_map.get(n, n): v for n, v in pos.items()}
        vel = {joint_map.get(n, n): v for n, v in vel.items()}
        eff = {joint_map.get(n, n): v for n, v in eff.items()}
    for name, value in (fixed or {}).items():
        pos.setdefault(name, value)
        vel.setdefault(name, 0.0)
        eff.setdefault(name, 0.0)
    missing = [name for name in names if name not in pos]
    if missing:
        raise ValueError(f"state lacks joints {missing}")
    sim_time = body.get("sim_time", payload.get("sim_time"))
    if sim_time is None:
        raise ValueError("state lacks sim_time")
    state = RobotState(sim_time=float(sim_time), positions=pos, velocities=vel, efforts=eff)
    base = body.get("base_pose") or body.get("base") or {}
    frame_axes = (base.get("frame") or body.get("frame_axes") or "game").lower()
    if base.get("position") is not None:
        state.base_position = (tuple(float(c) for c in base["position"]) if frame_axes == "ros"
                               else game_to_ros_vector(base["position"]))
    if base.get("orientation") is not None:
        state.base_orientation = (tuple(float(c) for c in base["orientation"]) if frame_axes == "ros"
                                  else game_to_ros_quaternion(base["orientation"]))
    if body.get("frame") is not None and isinstance(body.get("frame"), int):
        state.frame = int(body["frame"])
    for key in ("contacts", "beam_stress", "joint_loads", "broken_beams", "gripper_contact"):
        if key in body:
            state.extras[key] = body[key]
    return state


class McpBackend(Backend):
    name = "openbeam"

    def __init__(self, client: McpClient, robot: str = "", tools: Optional[Dict[str, str]] = None,
                 reconnect=None, joint_map: Optional[Dict[str, str]] = None,
                 fixed: Optional[Dict[str, float]] = None):
        """joint_map / fixed adapt a robot whose joints are named differently
        or that lacks some SO-101 joints (a test arm): debugging aids only."""
        self.joint_map = dict(joint_map or {})
        self.reverse_map = {ros: game for game, ros in self.joint_map.items()}
        self.fixed = dict(fixed or {})
        self.client = client
        self.robot = robot
        self.tools = dict(DEFAULT_TOOLS)
        self._image_overridden = bool((tools or {}).get("image"))
        self.tools.update({k: v for k, v in (tools or {}).items() if v})
        self._lock = threading.Lock()
        # Called after a transport failure to build a fresh client: a local
        # game that restarted has a new port and token in its endpoint file.
        self._reconnect = reconnect
        self._next_reconnect = 0.0
        self.camera_max_width = 640
        self.screenshot_keep_arg = False

    def _call(self, tool: str, args: Dict[str, Any]) -> Any:
        try:
            return self.client.call(tool, args)
        except McpError:
            if self._reconnect is not None and time.monotonic() >= self._next_reconnect:
                self._next_reconnect = time.monotonic() + 2.0
                with self._lock:
                    try:
                        client = self._reconnect()
                        client.initialize()
                        self.client = client
                    except Exception:
                        pass
            raise

    def _args(self, extra: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        args: Dict[str, Any] = {}
        if self.robot:
            args["robot"] = self.robot
        if extra:
            args.update(extra)
        return args

    def connect(self) -> Dict[str, Any]:
        info = self.client.initialize()
        tools = self.client.tools()
        missing = [self.tools[k] for k in ("state", "command") if self.tools[k] not in tools]
        if missing:
            raise McpError(
                f"the game does not offer {missing}; it needs OpenBeam's robot tools "
                "(see docs/GAME_CONTRACT.md). Available: " + ", ".join(sorted(tools)))
        schema = (tools.get("screenshot") or {}).get("inputSchema") or {}
        self.screenshot_keep_arg = "keep" in (schema.get("properties") or {})  # delete the file once returned
        if not self._image_overridden and "camera_frame" in tools:
            self.tools["image"] = "camera_frame"  # in-memory capture, no file per frame
        return {"server": info.get("serverInfo", {}), "tools": sorted(tools), "using": dict(self.tools)}

    def read_state(self) -> RobotState:
        started = time.monotonic()
        payload = self._call(self.tools["state"], self._args())
        state = parse_state(payload, joint_map=self.joint_map, fixed=self.fixed)
        state.received_monotonic = time.monotonic()
        state.request_seconds = state.received_monotonic - started
        return state

    def command_positions(self, positions: Dict[str, float]) -> Any:
        """Send targets; answers the tool's payload (a state, if the game's
        command tool returns one, which saves the separate read)."""
        clean = {self.reverse_map.get(name, name): so101.clamp(name, value)
                 for name, value in positions.items() if name in so101.BY_NAME and name not in self.fixed}
        if not clean:
            return None
        return self._call(self.tools["command"], self._args({"positions": clean}))

    def capture_image(self) -> Optional[Image]:
        started = time.monotonic()
        if self.tools["image"] == "camera_frame":
            args = {"max_width": self.camera_max_width, "format": "jpeg"}
        else:
            args = {"keep": False} if self.screenshot_keep_arg else {}
        mime, data, meta = self.client.image(self.tools["image"], args, with_meta=True)
        image = Image(mime=mime, data=data, request_seconds=time.monotonic() - started)
        if isinstance(meta, dict):
            if meta.get("sim_time") is not None:
                image.sim_time = float(meta["sim_time"])
            if meta.get("fov_y_deg") is not None:
                image.fov_y_deg = float(meta["fov_y_deg"])
            pose = meta.get("camera_pose") or {}
            if pose.get("position") is not None and pose.get("orientation") is not None:
                image.camera_position = game_to_ros_vector(pose["position"])
                # Ogre camera: looks along -Z, +Y up, +X right. In ROS axes that
                # is exactly camera_link (x forward, y left, z up), so the same
                # change of axes applies.
                image.camera_orientation = game_to_ros_quaternion(pose["orientation"])
        return image

    @property
    def camera_pose_live(self) -> bool:
        return self.tools["image"] == "camera_frame"


# --------------------------------------------------------------- the fake ---
class FakeBackend(Backend):
    """A kinematic SO-101 whose joints chase their targets like servos.

    Each joint is a critically damped second-order system (natural frequency
    `omega`) with a velocity limit, integrated on read, so it moves in real
    time without a thread.  Effort is a stand-in: k * error.
    """

    name = "fake"

    def __init__(self, omega: float = 12.0, max_velocity: float = 3.0, real_time_factor: float = 1.0,
                 clock=time.monotonic):
        self.omega = omega
        self.max_velocity = max_velocity
        self.rtf = real_time_factor
        self.clock = clock
        self._lock = threading.Lock()
        self._t0 = clock()
        self._last = self._t0
        self.sim_time = 0.0
        self.pos = {name: 0.0 for name in so101.JOINT_NAMES}
        self.vel = {name: 0.0 for name in so101.JOINT_NAMES}
        self.target = dict(self.pos)
        self.commands = 0

    def _advance(self) -> None:
        now = self.clock()
        dt_total = (now - self._last) * self.rtf
        self._last = now
        step = 0.002
        while dt_total > 1e-9:
            h = min(step, dt_total)
            dt_total -= h
            for name in so101.JOINT_NAMES:
                err = self.target[name] - self.pos[name]
                acc = self.omega ** 2 * err - 2.0 * self.omega * self.vel[name]
                v = self.vel[name] + acc * h
                v = max(-self.max_velocity, min(self.max_velocity, v))
                self.vel[name] = v
                self.pos[name] = so101.clamp(name, self.pos[name] + v * h)
            self.sim_time += h

    def read_state(self) -> RobotState:
        started = time.monotonic()
        with self._lock:
            self._advance()
            state = RobotState(
                sim_time=self.sim_time,
                positions=dict(self.pos),
                velocities=dict(self.vel),
                efforts={n: 2.0 * (self.target[n] - self.pos[n]) for n in so101.JOINT_NAMES},
                base_position=(0.0, 0.0, 0.0),
            )
        state.received_monotonic = time.monotonic()
        state.request_seconds = state.received_monotonic - started
        return state

    def command_positions(self, positions: Dict[str, float]) -> None:
        with self._lock:
            self._advance()
            for name, value in positions.items():
                if name in self.target:
                    self.target[name] = so101.clamp(name, value)
            self.commands += 1

    def capture_image(self) -> Optional[Image]:
        # A tiny valid PNG (8x8 grey) so the camera topics have something to carry.
        return Image(mime="image/png", data=_grey_png(8, 8), sim_time=self.sim_time)


def _grey_png(width: int, height: int) -> bytes:
    import struct
    import zlib

    raw = b"".join(b"\x00" + b"\x80\x80\x80" * width for _ in range(height))

    def chunk(kind: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF)

    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw)) + chunk(b"IEND", b""))


def make_backend(kind: str, **kwargs) -> Backend:
    if kind == "fake":
        return FakeBackend(**{k: v for k, v in kwargs.items() if k in ("omega", "max_velocity", "real_time_factor")})
    if kind in ("openbeam", "mcp", "game"):
        from .mcp_client import endpoint_from_env
        timeout = kwargs.get("timeout", 10.0)

        def build() -> McpClient:
            return McpClient(kwargs.get("endpoint") or endpoint_from_env(), timeout=timeout)

        return McpBackend(build(), robot=kwargs.get("robot", ""), tools=kwargs.get("tools"), reconnect=build,
                          joint_map=parse_joint_map(kwargs.get("joint_map", "")),
                          fixed=parse_fixed(kwargs.get("fixed_joints", "")))
    raise ValueError(f"unknown backend {kind!r} (fake | openbeam)")


__all__ = ["Backend", "FakeBackend", "McpBackend", "RobotState", "Image", "parse_state", "make_backend",
           "game_to_ros_vector", "game_to_ros_quaternion", "ToolError", "McpError", "math"]


# ------------------------------------------------------------ anchoring ---
def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quat_conjugate(q):
    return (-q[0], -q[1], -q[2], q[3])


def quat_rotate(q, v):
    x, y, z, _ = quat_multiply(quat_multiply(q, (v[0], v[1], v[2], 0.0)), quat_conjugate(q))
    return (x, y, z)


class WorldAnchor:
    """Re-express game-world poses relative to the robot base's first pose.

    A fixed-base arm sits at the origin of `world` in RViz instead of at
    (-511, -511, 0.3), while the arm's own motion and the camera keep their
    true placement relative to it.
    """

    def __init__(self, enabled: bool = True):
        self.enabled = enabled
        self.position = None
        self.inverse = None

    def apply(self, position, orientation):
        if not self.enabled:
            return tuple(position), tuple(orientation)
        if self.position is None:
            self.position = tuple(position)
            self.inverse = quat_conjugate(tuple(orientation))
        delta = tuple(p - o for p, o in zip(position, self.position))
        return quat_rotate(self.inverse, delta), quat_multiply(self.inverse, tuple(orientation))
