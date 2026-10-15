# openbeam-ros2

ROS 2 (Jazzy) packages that connect ROS to [OpenBeam](https://openbeam.world)'s
simulated SO-101 arm: `openbeam_bridge` (the bridge node) and
`openbeam_so101_description` (URDF, RViz config).
Repository: https://github.com/OpenBeam-OSS/openbeam-ros2

## Use OpenBeam from ROS 2 in 5 minutes

This repository holds a ROS 2 bridge for [OpenBeam](https://openbeam.world). It makes the simulated SO-101
arm look like a ros2_control arm, so ROS 2 code you already have can drive
it unchanged: trajectory publishers, `FollowJointTrajectory` clients
(MoveIt's interface), RViz and `tf2` listeners. It runs in Docker on
**ROS 2 Jazzy**, and Humble works too. The default base image is the
course's `tiryoh/ros2-desktop-vnc`.

```
 your ROS 2 nodes ──topics/actions──▶ openbeam_bridge ──MCP (HTTP)──▶ OpenBeam game
 RViz ◀── /joint_states /tf /camera ──┘   robot_state_publisher         (local or cloud)
```

## 1. Start a game (pick one)

**A. A game on this computer.** Start OpenBeam with its agent endpoint
turned on:

```bash
ROR_AGENT_PORT=0 ROR_AGENT_ENDPOINT_FILE=$HOME/openbeam-endpoint/agent-endpoint.json \
  /Applications/OpenBeam.app/Contents/MacOS/OpenBeam
```

Load a level and spawn the SO-101 arm.

**B. A cloud session.** Press **Play OpenBeam** on the Cloud dashboard and
keep the player tab open. Then sign this computer in once:

```bash
cd src/openbeam_bridge      # plain Python 3, no ROS needed
python3 -m openbeam_bridge.login --token-file ~/.config/openbeam/token
```

It prints a link and a code. Approve the code in the browser. The command
then lists your running sessions as `OPENBEAM_SESSION_ID=...`.

**C. No game.** Set `OPENBEAM_BACKEND=fake` to get a kinematic stand-in arm
that follows your commands. Use it to check your ROS side before you
connect a game.

## 2. Configure

```bash
cp .env.example .env      # in the repository root
# A: OPENBEAM_ENDPOINT_DIR=$HOME/openbeam-endpoint
# B: OPENBEAM_SESSION_ID=...   OPENBEAM_MCP_TOKEN_FILE=$HOME/.config/openbeam/token   STATE_INFLIGHT=8
# C: OPENBEAM_BACKEND=fake
```

## 3. Run the bridge and RViz

```bash
docker compose up --build
```

The image builds on the course's `tiryoh/ros2-desktop-vnc:jazzy`. If you
already use it, the build only adds a thin layer. If you don't, the first
pull is about 2.9 GB compressed and needs about 9 GB of Docker disk.

**Short on disk?** Build the lite image instead. It is `ros:jazzy-ros-base`
plus RViz, about 2.4 GB, and it has no browser desktop:

```bash
docker build --build-arg BASE_IMAGE=ros:jazzy-ros-base \
  --build-arg EXTRA_APT="xvfb ffmpeg x11-utils" -t openbeam/ros2-bridge:jazzy-lite .
```

Run the bridge from it with
`docker run --rm --entrypoint /opt/openbeam_ws/entrypoint.sh openbeam/ros2-bridge:jazzy-lite ros2 launch openbeam_bridge bridge.launch.py`
(plus the `OPENBEAM_*` variables and the endpoint mount from `docker-compose.yml`).
Use RViz on your host, or record it headless with `examples/record_demo.sh`.

Open <http://localhost:6080> to reach a desktop in the browser. RViz opens
there by itself, showing the arm, its TF frames and the game camera. Every
container in the compose project shares `ROS_DOMAIN_ID=42`.

## 4. Move the arm with stock ROS 2 code

ros2_controllers ships an example node that publishes trajectories. This
command runs it unchanged:

```bash
docker compose --profile demo up demo
```

Or try it by hand from a shell in the desktop (or `docker compose exec bridge bash`):

```bash
# one-shot positions: 5 arm joints (+ gripper), radians
ros2 topic pub --once /joint_commands std_msgs/msg/Float64MultiArray "{data: [0.5, -0.4, 0.6, 0.2, 0.0, 1.0]}"
# a trajectory, as joint_trajectory_controller takes it
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [shoulder_pan], points: [{positions: [1.0], time_from_start: {sec: 2}}]}"
# the action MoveIt uses
/opt/openbeam_ws/examples/send_follow_joint_trajectory.sh
# the gripper (radians; -0.17 closed .. 1.75 open)
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.2}}"
```

## Interfaces

| Topic / action | Type | Direction | Rate |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | out | each game frame, up to `state_rate_hz` (60) |
| `/clock` | `rosgraph_msgs/Clock` | out | with `/joint_states` |
| `/tf` | `world → base_link` (bridge); link frames (robot_state_publisher) | out | with `/joint_states` |
| `/tf_static` | fixed URDF joints; `world → openbeam_camera_link → openbeam_camera_optical` | out | latched |
| `/robot_description` | URDF (std_msgs/String, latched) | out | once |
| `/camera/image_raw`, `/camera/image_raw/compressed`, `/camera/camera_info` | `sensor_msgs` | out | `camera_rate_hz` (0.5 by default; see limits) |
| `/openbeam/extras` | `std_msgs/String` (JSON: contacts, joint loads, beam stress) | out | with `/joint_states`, when the game reports any |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | out | 1 Hz: measured rates, latency, errors |
| `/joint_trajectory_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | in | |
| `/joint_trajectory_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | action | |
| `/joint_commands` | `std_msgs/Float64MultiArray` (5 arm values, or 6 with the gripper) | in | |
| `/gripper_command` | `std_msgs/Float64` (rad) | in | |
| `/gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand` | action | |

The joints, in this order, are `shoulder_pan, shoulder_lift, elbow_flex,
wrist_flex, wrist_roll, gripper`, in radians. Zero is the middle of each
joint's range (the SO-ARM100 "new calibration"). Message stamps use
**simulated time**, so run your nodes with `use_sim_time:=true`. The
bridge's launch file already does this for robot_state_publisher and RViz.
Trajectories also play against simulated time: a game that runs slower than
real time slows the arm down with it.

Units for LeRobot users: the arm joints match LeRobot's names, with ROS in
radians and LeRobot datasets in degrees. The gripper's 0..100 maps linearly
onto -0.1745..1.745 rad (`openbeam_bridge.so101.gripper_rad_from_lerobot`).

## Measuring

```bash
ros2 topic hz /joint_states
ros2 run openbeam_bridge latency_probe --ros-args -p seconds:=10 -p steps:=10   # rates + command-to-motion latency
python3 -m openbeam_bridge.measure --seconds 10                                 # the game link alone, no ROS
```

Headless evidence (no desktop): build with `--build-arg EXTRA_APT="xvfb ffmpeg x11-utils"`,
then run `examples/record_demo.sh [fake|openbeam] [seconds]` in the image with `/out` mounted.
It records RViz to `rviz_demo.mp4`, takes screenshots, and writes topic rates and
`latency_probe.json`. Set `DEMO_COMMANDS=0` to observe without moving the arm.

## Limits worth knowing

- **State is frame-rate bound.** Every game tool call runs between two
  frames, so `/joint_states` cannot be fresher than the game's frame rate
  (about 60 Hz locally). Over the cloud, each read also pays the network
  round trip.
- **Over the cloud, keep several reads in flight.** Each call takes about
  140 ms there (the network round trip plus a frame). One read at a time
  gives about 7 Hz. `state_inflight:=8` reaches the session's frame rate
  (about 29 Hz in a 30 fps session), but each reading is still about
  140 ms old.
- **The camera.** With a game that offers `camera_frame`, the bridge
  captures the presented frame in memory (JPEG, `camera_max_width` 640) and
  follows the game camera's pose in TF. Older games only have `screenshot`:
  a full-window PNG of about 1 s and 3 MB each, written to disk. With those,
  keep `camera_rate_hz` at 0.5 or below.
- **Docker Desktop costs some rate.** Every game call is a new TCP connection
  (the game closes each one), and from a container on macOS each connection
  pays Docker's port forwarding. That measured 47-49 Hz from a container
  against 56 Hz from the host. RViz rendering in software in the same
  container brought it down to 28 Hz.
- **`world` is the arm's base.** By default (`world_anchor:=robot`) the
  bridge puts `world` at the robot base's first pose, so RViz shows the arm
  at the origin. The camera pose is expressed relative to it.
  `world_anchor:=game` uses the game's world origin instead.
- **Effort** is whatever the game reports for a joint. The SO-101 in
  OpenBeam is a node-and-beam arm, so it is the servo's torque, not a
  rigid-body inverse-dynamics value.

## What is in here

| Path | What |
|---|---|
| `src/openbeam_bridge/` | The bridge node, its ROS-free core (`core.py`, `trajectory.py`, `robot_api.py`, `mcp_client.py`), tests, `measure`, `latency_probe`, `login` |
| `src/openbeam_so101_description/` | The SO-101 URDF (TheRobotStudio SO-ARM100, Apache-2.0, pinned commit), meshes fetched and checksummed at build, RViz config |
| `docs/GAME_CONTRACT.md` | The two game tools the bridge needs, and the measurements behind the design |
| `examples/` | Goals for the stock trajectory publisher, a `FollowJointTrajectory` call |
| `Dockerfile`, `docker-compose.yml` | Jazzy image (desktop or headless), bridge + desktop + demo services |

Tests (no ROS needed): `python3 -m unittest discover -s src/openbeam_bridge/test`.
The image build runs them as well.

## Tests

```bash
python3 -m pytest src/openbeam_bridge/test        # ROS-free; or: colcon test --packages-select openbeam_bridge
```

## License

Apache-2.0. Copyright 2026 OpenBeam, LLC. See `LICENSE` and `NOTICE`; the
SO-101 description carries TheRobotStudio SO-ARM100's Apache-2.0 notice.
