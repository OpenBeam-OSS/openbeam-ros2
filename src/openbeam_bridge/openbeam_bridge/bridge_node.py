"""openbeam_bridge: an rclpy node that makes an OpenBeam SO-101 look like a
ros2_control arm.

Publishes   /joint_states, /clock, /tf (world -> base_link, camera),
            /camera/image_raw (+ /compressed) and /camera/camera_info,
            /openbeam/extras (contacts, beam stress: JSON), /diagnostics.
Subscribes  /joint_trajectory_controller/joint_trajectory (JointTrajectory),
            /joint_commands (Float64MultiArray, 5 or 6 values),
            /gripper_command (Float64, radians).
Actions     /joint_trajectory_controller/follow_joint_trajectory,
            /gripper_controller/gripper_cmd.

The links below base_link come from robot_state_publisher and the URDF in
openbeam_so101_description, as on a real arm.
"""

from __future__ import annotations

import io
import json
import math
import threading
import time

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from control_msgs.action import FollowJointTrajectory, GripperCommand
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, CompressedImage, Image as ImageMsg, JointState
from std_msgs.msg import Float64, Float64MultiArray, String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from trajectory_msgs.msg import JointTrajectory

from . import so101
from .core import BridgeCore, RateMeter
from .robot_api import RobotState, WorldAnchor, make_backend

try:
    from PIL import Image as PILImage  # python3-pil: decode the game's PNG for image_raw
except ImportError:  # pragma: no cover
    PILImage = None


def stamp(seconds: float) -> TimeMsg:
    msg = TimeMsg()
    whole = math.floor(seconds)
    msg.sec = int(whole)
    msg.nanosec = int(round((seconds - whole) * 1e9)) % 1_000_000_000
    return msg


def duration_seconds(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1e-9


class OpenBeamBridge(Node):
    def __init__(self):
        super().__init__("openbeam_bridge")
        p = self.declare_parameter
        p("backend", "openbeam")  # openbeam | fake
        p("robot", "")  # actor selector passed to the game tools ("" = the only SO-101)
        p("state_rate_hz", 60.0)
        p("command_rate_hz", 60.0)
        p("camera_rate_hz", 2.0)  # 0 turns the camera off
        p("camera_scale", 0.5)  # image_raw is downscaled by this factor
        p("camera_fov_y_deg", 60.0)
        p("camera_frame", "openbeam_camera_optical")
        p("camera_pose", [0.6, -0.6, 0.5, 0.0, 0.0, 0.0, 1.0])  # world -> camera_link when the game gives none
        p("world_frame", so101.WORLD_FRAME)
        p("base_frame", so101.BASE_FRAME)
        p("publish_world_tf", True)
        p("world_anchor", "robot")  # robot: world = the base's first pose; game: the game's world origin
        p("state_tool", "")
        p("command_tool", "")
        p("image_tool", "")
        p("state_inflight", 1)  # extra concurrent state reads; 3-6 for a cloud session
        p("goal_tolerance", 0.05)  # rad, for FollowJointTrajectory success
        p("request_timeout", 10.0)
        p("joint_map", "")  # debugging: "game_name:ros_name,..." for a robot named differently
        p("fixed_joints", "")  # debugging: "wrist_flex=0,..." for SO-101 joints the robot lacks
        g = lambda name: self.get_parameter(name).value  # noqa: E731

        self.world_frame = g("world_frame")
        self.base_frame = g("base_frame")
        self.camera_frame = g("camera_frame")
        self.publish_world_tf = bool(g("publish_world_tf"))
        self.goal_tolerance = float(g("goal_tolerance"))
        self.anchor = WorldAnchor(enabled=g("world_anchor") == "robot")
        backend = make_backend(
            g("backend"), robot=g("robot"), timeout=float(g("request_timeout")),
            joint_map=g("joint_map"), fixed_joints=g("fixed_joints"),
            tools={"state": g("state_tool"), "command": g("command_tool"), "image": g("image_tool")})
        self.core = BridgeCore(backend, command_rate_hz=float(g("command_rate_hz")),
                               on_state=self._publish_state, log=self.get_logger().warn)
        p("camera_max_width", 640)  # camera_frame downscale (game side)
        if hasattr(backend, "camera_max_width"):
            backend.camera_max_width = int(g("camera_max_width"))
        info = backend.connect()
        self.get_logger().info(f"backend {backend.name} connected: {json.dumps(info)[:300]}")

        cb = ReentrantCallbackGroup()
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.pub_joints = self.create_publisher(JointState, "joint_states", sensor_qos)
        self.pub_clock = self.create_publisher(Clock, "/clock", 10)
        self.pub_extras = self.create_publisher(String, "openbeam/extras", 10)
        self.pub_diag = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.pub_image = self.create_publisher(ImageMsg, "camera/image_raw", 2)
        self.pub_image_c = self.create_publisher(CompressedImage, "camera/image_raw/compressed", 2)
        self.pub_info = self.create_publisher(CameraInfo, "camera/camera_info", 2)
        self.tf = TransformBroadcaster(self)
        self.tf_static = StaticTransformBroadcaster(self)
        # A game with camera_frame reports where its camera is on every frame;
        # otherwise the camera link is a fixed, configured pose.
        self._publish_static_camera(None if backend.camera_pose_live else g("camera_pose"))

        self.create_subscription(JointTrajectory, "joint_trajectory_controller/joint_trajectory",
                                 self._on_trajectory, 10, callback_group=cb)
        self.create_subscription(Float64MultiArray, "joint_commands", self._on_joint_commands, 10, callback_group=cb)
        self.create_subscription(Float64, "gripper_command", self._on_gripper, 10, callback_group=cb)
        self._fjt = ActionServer(self, FollowJointTrajectory,
                                 "joint_trajectory_controller/follow_joint_trajectory",
                                 execute_callback=self._execute_fjt, goal_callback=self._accept_goal,
                                 cancel_callback=lambda _goal: CancelResponse.ACCEPT, callback_group=cb)
        self._grip = ActionServer(self, GripperCommand, "gripper_controller/gripper_cmd",
                                  execute_callback=self._execute_gripper, goal_callback=self._accept_goal,
                                  cancel_callback=lambda _goal: CancelResponse.ACCEPT, callback_group=cb)

        self.state_period = 1.0 / max(float(g("state_rate_hz")), 0.1)
        self.camera_rate = float(g("camera_rate_hz"))
        self.camera_scale = float(g("camera_scale"))
        self.fov_y = math.radians(float(g("camera_fov_y_deg")))
        self.camera_meter = RateMeter()
        self.publish_meter = RateMeter()
        self._stop = threading.Event()
        self._threads = [threading.Thread(target=self._state_loop, name="state", daemon=True)]
        if self.camera_rate > 0:
            self._threads.append(threading.Thread(target=self._camera_loop, name="camera", daemon=True))
        for index in range(max(0, int(g("state_inflight")) - 1)):
            self._threads.append(threading.Thread(target=self._reader_loop, args=(index + 1,),
                                                  name=f"reader{index + 1}", daemon=True))
        for thread in self._threads:
            thread.start()
        self.create_timer(1.0, self._publish_diagnostics, callback_group=cb)

    # ------------------------------------------------------------ loops ---
    def _state_loop(self) -> None:
        next_tick = time.monotonic()
        while not self._stop.is_set():
            self.core.tick()
            next_tick += self.state_period
            delay = next_tick - time.monotonic()
            if delay > 0:
                self._stop.wait(delay)
            else:
                next_tick = time.monotonic()  # fell behind: run as fast as the game answers

    def _reader_loop(self, index: int) -> None:
        # Staggered so the reads land on different frames.
        self._stop.wait(self.state_period * index / 2.0)
        while not self._stop.is_set():
            started = time.monotonic()
            self.core.read_only()
            self._stop.wait(max(0.0, self.state_period - (time.monotonic() - started)))

    def _camera_loop(self) -> None:
        period = 1.0 / self.camera_rate
        while not self._stop.is_set():
            started = time.monotonic()
            try:
                image = self.core.backend.capture_image()
                if image is not None:
                    self._publish_image(image)
                    self.camera_meter.tick(time.monotonic() - started)
            except Exception as error:
                self.get_logger().warn(f"camera: {error}", throttle_duration_sec=10.0)
            self._stop.wait(max(0.0, period - (time.monotonic() - started)))

    # ------------------------------------------------------- publishing ---
    def _publish_state(self, state: RobotState) -> None:
        t = stamp(state.sim_time)
        clock = Clock()
        clock.clock = t
        self.pub_clock.publish(clock)
        msg = JointState()
        msg.header.stamp = t
        msg.name = list(so101.JOINT_NAMES)
        msg.position = [state.positions[n] for n in so101.JOINT_NAMES]
        if all(n in state.velocities for n in so101.JOINT_NAMES):
            msg.velocity = [state.velocities[n] for n in so101.JOINT_NAMES]
        if all(n in state.efforts for n in so101.JOINT_NAMES):
            msg.effort = [state.efforts[n] for n in so101.JOINT_NAMES]
        self.pub_joints.publish(msg)
        if self.publish_world_tf:
            tf = TransformStamped()
            tf.header.stamp = t
            tf.header.frame_id = self.world_frame
            tf.child_frame_id = self.base_frame
            position, orientation = self.anchor.apply(state.base_position, state.base_orientation)
            (tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z) = position
            (tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z,
             tf.transform.rotation.w) = orientation
            self.tf.sendTransform(tf)
        if state.extras:
            self.pub_extras.publish(String(data=json.dumps({"sim_time": state.sim_time, **state.extras})))
        self.publish_meter.tick()

    def _publish_static_camera(self, pose) -> None:
        optical = TransformStamped()  # REP-103 camera_link (x fwd) -> optical (z fwd, x right, y down)
        optical.header.frame_id = "openbeam_camera_link"
        optical.child_frame_id = self.camera_frame
        (optical.transform.rotation.x, optical.transform.rotation.y, optical.transform.rotation.z,
         optical.transform.rotation.w) = (-0.5, 0.5, -0.5, 0.5)
        if pose is None:
            self.tf_static.sendTransform([optical])
            return
        values = [float(v) for v in pose]
        link = TransformStamped()
        link.header.frame_id = self.world_frame
        link.child_frame_id = "openbeam_camera_link"
        (link.transform.translation.x, link.transform.translation.y, link.transform.translation.z) = values[:3]
        (link.transform.rotation.x, link.transform.rotation.y, link.transform.rotation.z,
         link.transform.rotation.w) = values[3:7]
        self.tf_static.sendTransform([link, optical])

    def _publish_image(self, image) -> None:
        state = self.core.state
        t = stamp(image.sim_time if image.sim_time is not None else (state.sim_time if state else 0.0))
        fmt = "png" if "png" in image.mime else "jpeg"
        compressed = CompressedImage()
        compressed.header.stamp = t
        compressed.header.frame_id = self.camera_frame
        compressed.format = f"rgb8; {fmt} compressed rgb8"
        compressed.data = image.data
        self.pub_image_c.publish(compressed)
        if image.camera_position is not None and self.anchor.position is not None:
            position, orientation = self.anchor.apply(image.camera_position, image.camera_orientation)
            link = TransformStamped()
            link.header.stamp = t
            link.header.frame_id = self.world_frame
            link.child_frame_id = "openbeam_camera_link"
            (link.transform.translation.x, link.transform.translation.y,
             link.transform.translation.z) = position
            (link.transform.rotation.x, link.transform.rotation.y, link.transform.rotation.z,
             link.transform.rotation.w) = orientation
            self.tf.sendTransform(link)
        fov_y = math.radians(image.fov_y_deg) if image.fov_y_deg else self.fov_y
        width = height = 0
        if PILImage is not None:
            picture = PILImage.open(io.BytesIO(image.data)).convert("RGB")
            if 0 < self.camera_scale < 1.0:
                picture = picture.resize((max(1, int(picture.width * self.camera_scale)),
                                          max(1, int(picture.height * self.camera_scale))))
            width, height = picture.size
            raw = ImageMsg()
            raw.header = compressed.header
            raw.height, raw.width = height, width
            raw.encoding = "rgb8"
            raw.step = width * 3
            raw.data = picture.tobytes()
            self.pub_image.publish(raw)
        if width and height:
            info = CameraInfo()
            info.header = compressed.header
            info.width, info.height = width, height
            f = (height / 2.0) / math.tan(fov_y / 2.0)
            info.distortion_model = "plumb_bob"
            info.d = [0.0] * 5
            info.k = [f, 0.0, width / 2.0, 0.0, f, height / 2.0, 0.0, 0.0, 1.0]
            info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info.p = [f, 0.0, width / 2.0, 0.0, 0.0, f, height / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0]
            self.pub_info.publish(info)

    def _publish_diagnostics(self) -> None:
        core = self.core
        status = DiagnosticStatus()
        status.name = "openbeam_bridge"
        status.hardware_id = core.backend.name
        status.level = DiagnosticStatus.OK if core.state is not None else DiagnosticStatus.WARN
        status.message = core.last_error or "ok"
        fields = {
            "state_rate_hz": f"{core.state_rate.rate():.1f}",
            "state_latency_p50_ms": f"{core.state_rate.latency(0.5) * 1e3:.1f}",
            "state_latency_p95_ms": f"{core.state_rate.latency(0.95) * 1e3:.1f}",
            "command_rate_hz": f"{core.command_rate.rate():.1f}",
            "command_latency_p50_ms": f"{core.command_rate.latency(0.5) * 1e3:.1f}",
            "camera_rate_hz": f"{self.camera_meter.rate():.2f}",
            "camera_latency_p50_ms": f"{self.camera_meter.latency(0.5) * 1e3:.0f}",
            "sim_time": f"{core.state.sim_time:.3f}" if core.state else "",
            "errors": str(core.errors),
        }
        status.values = [KeyValue(key=k, value=v) for k, v in fields.items()]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.pub_diag.publish(array)

    # ---------------------------------------------------------- commands ---
    def _on_trajectory(self, msg: JointTrajectory) -> None:
        try:
            self._start_trajectory(msg)
        except Exception as error:
            self.get_logger().error(f"joint_trajectory refused: {error}")

    def _start_trajectory(self, msg: JointTrajectory):
        if not msg.points:
            self.core.cancel_trajectory()
            return None
        times = [duration_seconds(point.time_from_start) for point in msg.points]
        positions = [list(point.positions) for point in msg.points]
        velocities = [list(point.velocities) if len(point.velocities) == len(msg.joint_names) else None
                      for point in msg.points]
        if any(v is None for v in velocities):
            velocities = None
        return self.core.set_trajectory(list(msg.joint_names), times, positions, velocities)

    def _on_joint_commands(self, msg: Float64MultiArray) -> None:
        try:
            self.core.set_positions(so101.positions_from_array(list(msg.data)))
        except Exception as error:
            self.get_logger().error(f"joint_commands refused: {error}")

    def _on_gripper(self, msg: Float64) -> None:
        try:
            self.core.set_positions({so101.GRIPPER_JOINT: float(msg.data)})
        except Exception as error:
            self.get_logger().error(f"gripper_command refused: {error}")

    def _accept_goal(self, _goal) -> GoalResponse:
        return GoalResponse.ACCEPT if self.core.state is not None else GoalResponse.REJECT

    def _execute_fjt(self, goal_handle):
        result = FollowJointTrajectory.Result()
        try:
            trajectory = self._start_trajectory(goal_handle.request.trajectory)
        except Exception as error:
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            result.error_string = str(error)
            goal_handle.abort()
            return result
        if trajectory is None:
            goal_handle.succeed()
            return result
        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = list(trajectory.joint_names)
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.core.cancel_trajectory()
                goal_handle.canceled()
                return result
            if self.core.trajectory is not trajectory:
                break
            state = self.core.state
            if state is not None:
                feedback.header.stamp = stamp(state.sim_time)
                feedback.actual.positions = [state.positions[n] for n in trajectory.joint_names]
                feedback.desired.positions = [self.core.targets.get(n, 0.0) for n in trajectory.joint_names]
                feedback.error.positions = [a - d for a, d in zip(feedback.actual.positions,
                                                                  feedback.desired.positions)]
                goal_handle.publish_feedback(feedback)
            time.sleep(0.02)
        if self.core.trajectory is not None and self.core.trajectory is not trajectory:
            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            result.error_string = "pre-empted by a newer command"
            goal_handle.abort()
            return result
        # Let the servos settle onto the last point, then judge the goal tolerance.
        final = dict(zip(trajectory.joint_names, trajectory.points[-1].positions))
        settle_until = time.monotonic() + max(1.0, duration_seconds(goal_handle.request.goal_time_tolerance))
        worst = math.inf
        while time.monotonic() < settle_until:
            state = self.core.state
            if state is not None:
                worst = max(abs(state.positions[n] - so101.clamp(n, v)) for n, v in final.items())
                if worst <= self.goal_tolerance:
                    break
            time.sleep(0.02)
        if worst <= self.goal_tolerance:
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            goal_handle.succeed()
        else:
            result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
            result.error_string = f"final error {worst:.3f} rad > {self.goal_tolerance}"
            goal_handle.abort()
        return result

    def _execute_gripper(self, goal_handle):
        target = so101.clamp(so101.GRIPPER_JOINT, float(goal_handle.request.command.position))
        self.core.set_positions({so101.GRIPPER_JOINT: target})
        result = GripperCommand.Result()
        deadline = time.monotonic() + 3.0
        last = None
        stalled_since = None
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return result
            state = self.core.state
            if state is not None:
                pos = state.positions[so101.GRIPPER_JOINT]
                result.position = pos
                result.effort = state.efforts.get(so101.GRIPPER_JOINT, 0.0)
                if abs(pos - target) <= self.goal_tolerance:
                    result.reached_goal = True
                    break
                if last is not None and abs(pos - last) < 1e-4:
                    stalled_since = stalled_since or time.monotonic()
                    if time.monotonic() - stalled_since > 0.5:
                        result.stalled = True  # closed on an object
                        break
                else:
                    stalled_since = None
                last = pos
            time.sleep(0.02)
        goal_handle.succeed()
        return result

    def destroy_node(self):
        self._stop.set()
        for thread in self._threads:
            thread.join(timeout=2.0)
        self.core.backend.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OpenBeamBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
