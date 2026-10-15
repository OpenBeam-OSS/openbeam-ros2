#!/bin/bash
# Headless demo + evidence: bridge, RViz on a virtual display, the stock
# ros2_controllers trajectory publisher, a video and topic measurements.
# Needs an image built with --build-arg EXTRA_APT="xvfb ffmpeg x11-utils".
#   docker run --rm -v $PWD/out:/out [-e OPENBEAM_...] openbeam/ros2-bridge:jazzy-lite \
#     /opt/openbeam_ws/examples/record_demo.sh [backend] [seconds]
# Runs inside its own container: the pkill calls below only see this demo.
BACKEND=${1:-fake}
RECORD=${2:-30}
OUT=${OUT:-/out}
mkdir -p "$OUT"
source /opt/ros/${ROS_DISTRO}/setup.bash
source /opt/openbeam_ws/install/setup.bash
export DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1
Xvfb :99 -screen 0 1400x900x24 >/dev/null 2>&1 &
sleep 2
ros2 launch openbeam_bridge bridge.launch.py backend:="$BACKEND" rviz:=true \
  camera_rate_hz:=${CAMERA_RATE_HZ:-0.5} ${BRIDGE_ARGS:-} > "$OUT/bridge.log" 2>&1 &
sleep 10
ros2 topic echo --csv /joint_states sensor_msgs/msg/JointState > "$OUT/joint_states.csv" 2>/dev/null &
if [ "${DEMO_COMMANDS:-1}" = 1 ]; then
  ros2 run ros2_controllers_test_nodes publisher_joint_trajectory_controller \
    --ros-args --params-file /opt/openbeam_ws/examples/so101_trajectory_goals.yaml > "$OUT/publisher.log" 2>&1 &
fi
ffmpeg -loglevel error -y -f x11grab -video_size 1400x900 -framerate 15 -i :99 -t "$RECORD" \
  -pix_fmt yuv420p "$OUT/rviz_demo.mp4" &
ffpid=$!
step=$((RECORD / 4))
for i in 1 2 3; do
  sleep "$step"
  ffmpeg -loglevel error -y -f x11grab -video_size 1400x900 -i :99 -frames:v 1 "$OUT/rviz_$((i * step))s.png"
done
wait "$ffpid"
pkill -f publisher_joint_trajectory_controller   # the probe needs the arm to itself
pkill -f "topic echo --csv"
sleep 2
measure=()
for topic in /joint_states /clock /tf; do
  timeout 15 ros2 topic hz "$topic" --window 300 > "$OUT/hz_${topic//\//}.txt" 2>&1 & measure+=($!)
done
ros2 run openbeam_bridge latency_probe --ros-args -p seconds:=8.0 -p steps:=$([ "${DEMO_COMMANDS:-1}" = 1 ] && echo 10 || echo 0) \
  > "$OUT/latency_probe.json" 2> "$OUT/latency_probe.err"
wait "${measure[@]}"
ros2 topic list -t > "$OUT/topics.txt"
ros2 action list -t > "$OUT/actions.txt"
timeout 5 ros2 topic echo --once /diagnostics > "$OUT/diagnostics.txt" 2>&1
pkill -f "ros2 launch"
sleep 2
echo "evidence in $OUT"
