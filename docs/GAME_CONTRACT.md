# What the ROS 2 bridge needs from the game

**Status:** OpenBeam's robot tools implement `robot_state`,
`robot_set_joints`, `camera_frame` and a `screenshot` `keep` argument, all
matching this contract. They also offer `robot_info/fk/ik/jacobian`, plus
`sim_mode`/`sim_step` for stepped physics. OpenBeam's SO-101 arm publishes
the state those tools read.

The bridge only talks MCP: the same `POST /mcp` that a local game serves with
`ROR_AGENT_PORT`, and that the cloud relays at
`/v1/sessions/{id}/mcp`. It needs two robot tools beyond the stock set. They
belong to the game's robot tools. The
names below are the bridge's defaults. If the game names them differently,
set the bridge parameters `state_tool`, `command_tool` and `image_tool`.

## Why polling MCP is enough (measured)

The game runs every MCP tool call on its main thread, between frames.
Measured on an M-series Mac with the installed OpenBeam 2026.10.3 (simple2
terrain, 60 Hz display):

| Measurement | Result |
|---|---|
| One `status` call after another | 57.6 calls/s, p50 17.5 ms, p95 19.1 ms (one frame each) |
| 2 or 4 calls in parallel on separate connections | still 57.7 rounds/s: all are served in the same frame |
| A JSON-RPC batch of 2 calls | 34.9 ms: a batch runs one call per frame |
| `screenshot` (2560x1440 PNG, 3.0 MB) | 1.07 s each, ~0.9/s at most; frames are not stalled (status max 59 ms meanwhile) |

Against a live game with the robot tools (a 4-joint test arm), from the
host: `robot_state` ran at 56 Hz (p50 17.5 ms), and
`robot_set_joints` took one frame. `camera_frame` (640 px JPEG, 31 KB) took
140 ms at p50, about 6.7 Hz. From Docker Desktop on macOS, the bridge
reached 47-49 Hz. Each new TCP connection pays Docker's port forwarding.

**Cloud** (2026-10-15, an ordinary session placed by the scheduler, test
account, `/v1/sessions/{id}/mcp`, from a home connection with 62 ms RTT;
production game `49a858eee`, which has no robot tools yet, so `status`
stands in for the state read and `run_script` for the command):

| Calls in flight | Calls/s | Distinct game frames/s | p50 / p95 per call |
|---|---|---|---|
| 1 | 6.8 | 6.8 | 138 / 205 ms |
| 2 | 13.5 | 9.1 | 139 / 207 ms |
| 4 | 27.0 | 17.7 | 137 / 212 ms |
| 6 | 40.0 | 23.7 | 139 / 209 ms |
| 8 | 49.3 | 28.8 | 156 / 215 ms |

The command proxy gave 6.9 calls/s with one call in flight and 28.1 with
four. `screenshot` (1280x720 PNG, 750 KB) took 0.79 s. The session ran at
30 fps, as requested, so about 29 distinct frames/s is the ceiling. Six to
eight reads in flight reach it (`state_inflight:=8`). Every reading is
still about 140 ms old. That is fine for monitoring and trajectory
streaming, but too slow for a tight closed loop over the internet. The
stream was not attached during these runs (headless Chromium has no
WebTransport), so stream-encoding load was absent. All three sessions were
ended.

So a state read costs one frame. The bridge sends the command and the state
read on two connections at once, so a control tick also costs one frame.
State is therefore as fresh as the game's frame rate allows: about 60 Hz.
A push stream could not deliver fresher state without exposing physics
sub-steps, so none is needed for joint state. The camera is the exception
(see `camera_frame` below).

## `robot_state` (required)

Arguments: `{"robot": "<selector>"}`. `robot` is optional when the session
holds exactly one robot. The selector is the actor instance id or the truck
file name.

Answer (`structuredContent`):

```json
{
  "sim_time": 12.345,
  "frame": 7410,
  "robot": {"instance_id": 3, "file": "so101.jbeam"},
  "joints": [
    {"name": "shoulder_pan",  "position": 0.12, "velocity": 0.0, "effort": 0.31},
    {"name": "shoulder_lift", "position": -0.4, "velocity": 0.0, "effort": 1.2},
    {"name": "elbow_flex",    "position": 0.5,  "velocity": 0.0, "effort": 0.9},
    {"name": "wrist_flex",    "position": 0.3,  "velocity": 0.0, "effort": 0.1},
    {"name": "wrist_roll",    "position": 0.0,  "velocity": 0.0, "effort": 0.0},
    {"name": "gripper",       "position": 0.8,  "velocity": 0.0, "effort": 0.05}
  ],
  "base_pose": {"position": [x, y, z], "orientation": [qx, qy, qz, qw]},
  "contacts": [{"link": "moving_jaw_so101_v1_link", "position": [x, y, z], "normal": [x, y, z], "force": 2.1}],
  "joint_loads": {"shoulder_lift": {"torque": 1.2, "limit": 2.94}},
  "broken_beams": 0
}
```

- `sim_time`, in seconds, is simulated time since the level loaded. It only
  advances while physics runs, and it stops when the game is paused. It
  drives `/clock`, every message stamp, and trajectory playback.
- The joint `name`s are the SO-101 table in OpenBeam's SO-101 documentation:
  `shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper`.
- Units: `position` in rad, `velocity` in rad/s, `effort` in N·m. Positions
  follow the URDF's "new calibration" convention: zero is mid-range. The
  gripper is also in rad (URDF range -0.1745..1.745); it is not LeRobot's
  0..100.
- `base_pose` is the `base_link` frame in game world axes (+Y up, -Z
  forward); the bridge converts it to REP-103. A game that already reports
  ROS axes says `"frame": "ros"` inside `base_pose`.
- Optional extras (`contacts`, `joint_loads`, `beam_stress`,
  `broken_beams`, `gripper_contact`) are forwarded as JSON on
  `/openbeam/extras`.

The bridge also accepts `joints` as `{name: position}` or
`{name: {position, velocity, effort}}`, and parallel `name` / `position` /
`velocity` / `effort` arrays in the style of `sensor_msgs/JointState`.

## `robot_set_joints` (required)

Arguments: `{"robot": "<selector>", "positions": {"shoulder_pan": 0.5, "gripper": 1.0}}`.

- The answer gives the targets in force from the frame that serviced the
  call, and they are held until changed. Joints that are not named keep
  their targets.
- Values outside a joint's limits are clamped, and the answer reports them:
  `{"accepted": [...], "clamped": {"gripper": 1.745}}`.
- Optional: answer the post-command state in the `robot_state` shape. The
  bridge then skips its parallel state read.

## `camera_frame`

`screenshot` writes a full-resolution PNG to the screenshots folder and then
returns it. That means about 1 s and 3 MB per frame. The files are never
deleted, so a 0.5 Hz camera grows the disk by about 5 GB per hour. The
bridge therefore defaults the camera to 0.5 Hz, and the README tells users
to keep it low. A ROS-grade camera needs an in-memory capture instead:

`camera_frame {"max_width": 640, "format": "jpeg"}`. It
returns the most recently presented frame, downscaled and encoded in
memory, as image content. No file is written. Its `structuredContent`
carries `{"sim_time", "frame", "width", "height", "fov_y_deg",
"camera_pose"}`. The bridge picks `camera_frame` automatically when the
game offers it (`image_tool` overrides). The bridge then publishes
`world → openbeam_camera_link` from `camera_pose` on every frame, and
takes `fov_y_deg` for `camera_info`. When the bridge falls back to
`screenshot`, it passes `keep: false` if the game accepts it, so no files
pile up.

## Stepped physics (not used by the bridge)

`sim_mode {"mode": "stepped"}` and `sim_step {"steps", "positions"}` let a
client advance physics in exact 0.5 ms steps. Use them for RL or for
deterministic replay. The bridge stays real-time, as a ROS robot driver
does. A course that wants lockstep should call `sim_step` from its own
environment ([openbeam-gym](https://github.com/OpenBeam-OSS/openbeam-gym)), not through ROS.
In the cloud, stepped physics is allowed only in training sessions.

Found while measuring on the 2026.10.3 build: `app_screenshot_format=jpg`
wrote 0-byte `.jpg` files, and the MCP `screenshot` call timed out. Builds
with the robot tools write JPEG captures as RGB.
