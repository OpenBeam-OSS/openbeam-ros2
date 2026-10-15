"""ROS-free tests: run with `python3 -m pytest src/openbeam_bridge/test`
(or `python3 -m unittest discover -s ... -p 'test_*.py'`)."""

import http.server
import json
import math
import os
import sys
import tempfile
import threading
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from openbeam_bridge import so101  # noqa: E402
from openbeam_bridge.core import BridgeCore, RateMeter  # noqa: E402
from openbeam_bridge.mcp_client import McpClient, Endpoint, ToolError, endpoint_from_env  # noqa: E402
from openbeam_bridge.robot_api import (FakeBackend, McpBackend, game_to_ros_quaternion,  # noqa: E402
                                       game_to_ros_vector, parse_state)
from openbeam_bridge.trajectory import Trajectory, TrajectoryPoint, from_lists  # noqa: E402


class FakeClock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


class TrajectoryTests(unittest.TestCase):
    def test_linear_from_current_state(self):
        traj = from_lists(["shoulder_pan"], [1.0], [[1.0]], start_time=10.0, current={"shoulder_pan": 0.0})
        self.assertAlmostEqual(traj.sample(10.0)["shoulder_pan"], 0.0)
        self.assertAlmostEqual(traj.sample(10.5)["shoulder_pan"], 0.0 + 0.5 * 1.0, places=6)
        self.assertAlmostEqual(traj.sample(12.0)["shoulder_pan"], 1.0)
        self.assertTrue(traj.done(11.0))

    def test_hermite_passes_through_points_with_given_velocity(self):
        traj = Trajectory(["elbow_flex"], [TrajectoryPoint(0.0, [0.0], [0.0]), TrajectoryPoint(2.0, [1.0], [0.0])],
                          start_time=0.0)
        self.assertAlmostEqual(traj.sample(1.0)["elbow_flex"], 0.5, places=6)
        # zero end velocities: slow near the ends, so the quarter point is below linear
        self.assertLess(traj.sample(0.5)["elbow_flex"], 0.25)

    def test_rejects_bad_input(self):
        with self.assertRaises(ValueError):
            Trajectory(["a", "a"], [TrajectoryPoint(0.0, [0, 0])], 0.0)
        with self.assertRaises(ValueError):
            Trajectory(["a"], [TrajectoryPoint(1.0, [0]), TrajectoryPoint(0.5, [1])], 0.0, {"a": 0})
        with self.assertRaises(ValueError):
            Trajectory(["a"], [TrajectoryPoint(1.0, [0])], 0.0)  # no current position


class So101Tests(unittest.TestCase):
    def test_names_match_upstream_urdf(self):
        self.assertEqual(so101.JOINT_NAMES,
                         ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"])

    def test_urdf_limits_match_table(self):
        import xml.etree.ElementTree as ET
        urdf = os.path.join(os.path.dirname(__file__), "..", "..", "openbeam_so101_description", "urdf",
                            "so101.urdf")
        root = ET.parse(urdf).getroot()
        found = {}
        for joint in root.findall("joint"):
            if joint.get("type") == "revolute":
                limit = joint.find("limit")
                found[joint.get("name")] = (float(limit.get("lower")), float(limit.get("upper")))
        self.assertEqual(sorted(found), sorted(so101.JOINT_NAMES))
        for joint in so101.JOINTS:
            self.assertAlmostEqual(found[joint.name][0], joint.lower, places=5)
            self.assertAlmostEqual(found[joint.name][1], joint.upper, places=5)

    def test_array_commands_and_clamping(self):
        cmd = so101.positions_from_array([0, 0, 0, 0, 0])
        self.assertEqual(list(cmd), so101.ARM_JOINT_NAMES)
        cmd = so101.positions_from_array([10, 0, 0, 0, 0, 0])
        self.assertAlmostEqual(cmd["shoulder_pan"], 1.91986)
        with self.assertRaises(ValueError):
            so101.positions_from_array([0, 0])
        with self.assertRaises(ValueError):
            so101.positions_from_array([float("nan")] * 5)

    def test_gripper_units(self):
        self.assertAlmostEqual(so101.gripper_rad_from_lerobot(0), -0.174533)
        self.assertAlmostEqual(so101.gripper_rad_from_lerobot(100), 1.74533)
        self.assertAlmostEqual(so101.gripper_lerobot_from_rad(so101.gripper_rad_from_lerobot(37.0)), 37.0)


class AxesTests(unittest.TestCase):
    def test_game_forward_is_ros_x(self):
        self.assertEqual(game_to_ros_vector((0, 0, -1)), (1.0, -0.0, 0.0))
        self.assertEqual(game_to_ros_vector((0, 1, 0)), (-0.0, -0.0, 1.0))
        self.assertEqual(game_to_ros_vector((-1, 0, 0)), (-0.0, 1.0, 0.0))  # game left is ROS +y

    def test_yaw_about_game_up_is_yaw_about_ros_z(self):
        half = math.radians(30) / 2
        q = (0.0, math.sin(half), 0.0, math.cos(half))  # +30 deg about game +Y
        x, y, z, w = game_to_ros_quaternion(q)
        self.assertAlmostEqual(z, math.sin(half))
        self.assertAlmostEqual(w, math.cos(half))
        self.assertAlmostEqual(x, 0.0)
        self.assertAlmostEqual(y, 0.0)


class AnchorTests(unittest.TestCase):
    def test_base_at_origin_and_camera_relative(self):
        from openbeam_bridge.robot_api import WorldAnchor
        half = math.radians(90) / 2
        yaw90 = (0.0, 0.0, math.sin(half), math.cos(half))
        anchor = WorldAnchor()
        p, q = anchor.apply((-511.0, -511.0, 0.3), yaw90)
        self.assertEqual([round(c, 9) for c in p], [0, 0, 0])
        self.assertAlmostEqual(q[3], 1.0)
        # a camera 1 m along the base's +x (world -y after the 90 deg yaw)
        p, _ = anchor.apply((-511.0, -510.0, 1.3), yaw90)
        self.assertAlmostEqual(p[0], 1.0)
        self.assertAlmostEqual(p[1], 0.0)
        self.assertAlmostEqual(p[2], 1.0)
        off = WorldAnchor(enabled=False)
        self.assertEqual(off.apply((1, 2, 3), (0, 0, 0, 1))[0], (1, 2, 3))


class ParseStateTests(unittest.TestCase):
    def test_contract_shape(self):
        payload = {"sim_time": 1.5, "frame": 90,
                   "joints": [{"name": n, "position": 0.1, "velocity": 0.0, "effort": 0.2} for n in so101.JOINT_NAMES],
                   "base_pose": {"position": [1, 2, 3], "orientation": [0, 0, 0, 1]},
                   "contacts": [{"link": "gripper_link", "force": 3.0}]}
        state = parse_state(payload)
        self.assertEqual(state.sim_time, 1.5)
        self.assertEqual(state.frame, 90)
        self.assertEqual(state.base_position, (-3.0, -1.0, 2.0))
        self.assertIn("contacts", state.extras)

    def test_variants(self):
        names = so101.JOINT_NAMES
        state = parse_state({"sim_time": 0, "joints": {n: 0.5 for n in names}})
        self.assertEqual(state.positions["gripper"], 0.5)
        state = parse_state({"sim_time": 0, "name": names, "position": [0.0] * 6, "velocity": [1.0] * 6})
        self.assertEqual(state.velocities["wrist_roll"], 1.0)
        with self.assertRaises(ValueError):
            parse_state({"sim_time": 0, "joints": {"shoulder_pan": 0}})
        with self.assertRaises(ValueError):
            parse_state({"joints": {n: 0 for n in names}})


class JointMapTests(unittest.TestCase):
    def test_rename_and_fill_then_reverse_on_command(self):
        from openbeam_bridge.robot_api import parse_fixed, parse_joint_map

        joint_map = parse_joint_map("base_pan:shoulder_pan, elbow:elbow_flex")
        fixed = parse_fixed("shoulder_lift=0,wrist_flex=0.1,wrist_roll=0,gripper=0")
        state = parse_state({"sim_time": 0, "joints": [{"name": "base_pan", "position": 0.2},
                                                       {"name": "elbow", "position": -0.3}]},
                            joint_map=joint_map, fixed=fixed)
        self.assertEqual(state.positions["shoulder_pan"], 0.2)
        self.assertEqual(state.positions["wrist_flex"], 0.1)
        sent = []

        class Client:
            def call(self, tool, args):
                sent.append((tool, args))

        backend = McpBackend(Client(), joint_map=joint_map, fixed=fixed)
        backend.command_positions({"shoulder_pan": 0.5, "elbow_flex": 0.1, "gripper": 1.0})
        self.assertEqual(sent, [("robot_set_joints", {"positions": {"base_pan": 0.5, "elbow": 0.1}})])
        with self.assertRaises(ValueError):
            parse_joint_map("base_pan:not_a_joint")


class FakeBackendTests(unittest.TestCase):
    def test_servo_reaches_target(self):
        clock = FakeClock()
        robot = FakeBackend(clock=clock)
        robot.command_positions({"shoulder_pan": 0.5, "gripper": 1.0})
        clock.t += 2.0
        state = robot.read_state()
        self.assertAlmostEqual(state.positions["shoulder_pan"], 0.5, places=3)
        self.assertAlmostEqual(state.positions["gripper"], 1.0, places=3)
        self.assertAlmostEqual(state.sim_time, 2.0, places=6)


class CoreTests(unittest.TestCase):
    def test_trajectory_plays_against_sim_time(self):
        clock = FakeClock()
        robot = FakeBackend(clock=clock)
        published = []
        core = BridgeCore(robot, command_rate_hz=0, on_state=published.append, log=lambda *_: None)
        core.tick()
        done = []
        core.trajectory_done_callbacks.append(done.append)
        core.set_trajectory(["shoulder_pan", "elbow_flex"], [1.0, 2.0], [[0.5, -0.5], [1.0, -1.0]])
        for _ in range(300):
            clock.t += 0.01
            core.tick()
        self.assertEqual(done, [True])
        self.assertIsNone(core.trajectory)
        self.assertAlmostEqual(core.state.positions["shoulder_pan"], 1.0, places=2)
        self.assertAlmostEqual(core.state.positions["elbow_flex"], -1.0, places=2)
        self.assertGreater(robot.commands, 100)  # streamed, not one jump
        self.assertEqual(len(published), 301)

    def test_direct_command_preempts_trajectory(self):
        clock = FakeClock()
        core = BridgeCore(FakeBackend(clock=clock), command_rate_hz=0, log=lambda *_: None)
        core.tick()
        done = []
        core.trajectory_done_callbacks.append(done.append)
        core.set_trajectory(["shoulder_pan"], [5.0], [[1.0]])
        core.set_positions({"shoulder_pan": -0.3})
        self.assertEqual(done, [False])
        with self.assertRaises(ValueError):
            core.set_positions({"elbow": 0.0})

    def test_command_rate_limit(self):
        clock = FakeClock()
        robot = FakeBackend(clock=clock)
        core = BridgeCore(robot, command_rate_hz=10, log=lambda *_: None)
        core.tick()
        core.set_trajectory(["shoulder_pan"], [1.0], [[1.0]])
        start = time.monotonic()
        while time.monotonic() - start < 0.35:
            clock.t += 0.001
            core.tick()
        self.assertLessEqual(robot.commands, 5)

    def test_tick_survives_backend_errors(self):
        class Broken(FakeBackend):
            def read_state(self):
                raise ConnectionError("game went away")

        core = BridgeCore(Broken(), log=lambda *_: None)
        self.assertIsNone(core.tick())
        self.assertEqual(core.errors, 1)
        self.assertIn("game went away", core.last_error)

    def test_pipelined_reads_never_publish_older_or_repeated_states(self):
        from openbeam_bridge.robot_api import RobotState
        published = []
        core = BridgeCore(FakeBackend(), on_state=published.append, log=lambda *_: None)
        zero = {n: 0.0 for n in so101.JOINT_NAMES}
        for t in (1.0, 3.0, 2.0, 3.0, 4.0):
            core._accept(RobotState(sim_time=t, positions=zero))
        self.assertEqual([s.sim_time for s in published], [1.0, 3.0, 4.0])

    def test_rate_meter(self):
        meter = RateMeter(window=10)
        for i in range(11):
            meter.tick(latency=i / 1000.0, now=i * 0.1)
        self.assertAlmostEqual(meter.rate(now=1.0), 10.0)
        self.assertAlmostEqual(meter.latency(0.5), 0.005)


class _FakeGame(http.server.BaseHTTPRequestHandler):
    """Speaks the subset of the game's MCP the bridge uses."""
    token = "t" * 32
    robot = None
    calls = []

    def log_message(self, *_):
        pass

    def do_POST(self):
        if self.headers.get("Authorization") != f"Bearer {self.token}":
            self.send_response(401)
            self.end_headers()
            return
        body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
        if isinstance(body, list):
            out = [self.answer(m) for m in body]
        else:
            out = self.answer(body)
        if out is None:
            self.send_response(202)
            self.end_headers()
            return
        data = json.dumps(out).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Connection", "close")
        self.end_headers()
        self.wfile.write(data)

    def answer(self, message):
        if "id" not in message:
            return None
        method = message["method"]
        if method == "initialize":
            result = {"protocolVersion": "2025-06-18", "serverInfo": {"name": "openbeam", "version": "test"}}
        elif method == "tools/list":
            result = {"tools": [{"name": n} for n in ("status", "robot_state", "robot_set_joints", "screenshot", "camera_frame")]}
        else:
            name = message["params"]["name"]
            args = message["params"]["arguments"]
            type(self).calls.append((name, args))
            if name == "robot_state":
                state = self.robot.read_state()
                payload = {"sim_time": state.sim_time,
                           "joints": [{"name": n, "position": state.positions[n]} for n in so101.JOINT_NAMES]}
                result = {"structuredContent": payload, "content": [{"type": "text", "text": json.dumps(payload)}]}
            elif name == "robot_set_joints":
                self.robot.command_positions(args["positions"])
                result = {"structuredContent": {"accepted": sorted(args["positions"])}}
            elif name == "camera_frame":
                import base64
                from openbeam_bridge.robot_api import _grey_png
                meta = {"sim_time": 3.5, "fov_y_deg": 45.0, "width": 8, "height": 8,
                        "camera_pose": {"position": [0, 1, 2], "orientation": [0, 0, 0, 1], "frame": "world"}}
                result = {"structuredContent": meta, "content": [
                    {"type": "image", "mimeType": "image/jpeg", "data": base64.b64encode(_grey_png(8, 8)).decode()},
                    {"type": "text", "text": json.dumps(meta)}]}
            else:
                result = {"isError": True, "structuredContent": {"error": "nope", "message": "unknown"}}
        return {"jsonrpc": "2.0", "id": message["id"], "result": result}


class McpBackendTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        _FakeGame.robot = FakeBackend()
        cls.server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), _FakeGame)
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()
        cls.url = f"http://127.0.0.1:{cls.server.server_address[1]}/mcp"

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()

    def test_round_trip(self):
        backend = McpBackend(McpClient(Endpoint(self.url, _FakeGame.token)), robot="so101")
        info = backend.connect()
        self.assertIn("robot_state", info["tools"])
        backend.command_positions({"shoulder_pan": 0.25, "bogus": 1.0})
        self.assertEqual(_FakeGame.calls[-1], ("robot_set_joints", {"robot": "so101",
                                                                    "positions": {"shoulder_pan": 0.25}}))
        state = backend.read_state()
        self.assertIn("gripper", state.positions)
        with self.assertRaises(ToolError):
            backend.client.call("status")
        # camera_frame is preferred when offered, and its metadata reaches the image
        self.assertEqual(backend.tools["image"], "camera_frame")
        self.assertTrue(backend.camera_pose_live)
        image = backend.capture_image()
        self.assertEqual(_FakeGame.calls[-1], ("camera_frame", {"max_width": 640, "format": "jpeg"}))
        self.assertEqual(image.sim_time, 3.5)
        self.assertEqual(image.fov_y_deg, 45.0)
        self.assertEqual(image.camera_position, (-2.0, -0.0, 1.0))

    def test_bad_token_is_a_clear_error(self):
        client = McpClient(Endpoint(self.url, "wrong"))
        with self.assertRaisesRegex(Exception, "401"):
            client.initialize()

    def test_endpoint_resolution(self):
        with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as handle:
            json.dump({"host": "127.0.0.1", "port": 4567, "token": "abc", "mcpPath": "/mcp"}, handle)
        try:
            ep = endpoint_from_env({"OPENBEAM_ENDPOINT_FILE": handle.name, "OPENBEAM_ENDPOINT_HOST": "host.docker.internal"})
            self.assertEqual(ep.url, "http://host.docker.internal:4567/mcp")
            self.assertEqual(ep.token, "abc")
        finally:
            os.unlink(handle.name)
        ep = endpoint_from_env({"OPENBEAM_SESSION_ID": "abc-1", "OPENBEAM_MCP_TOKEN": "x"})
        self.assertEqual(ep.url, "https://cloud.openbeam.world/v1/sessions/abc-1/mcp")
        self.assertNotIn("x", ep.redacted())


if __name__ == "__main__":
    unittest.main()
