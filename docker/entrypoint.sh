#!/bin/bash
# Run a command inside the sourced ROS 2 + OpenBeam workspace.
set -e
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /opt/openbeam_ws/install/setup.bash
exec "$@"
