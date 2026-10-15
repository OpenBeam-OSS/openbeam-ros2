# OpenBeam ROS 2 bridge + SO-101 description.
#
# Default base: the course's desktop image (RViz in a browser via noVNC), so
# one image serves the bridge, RViz and the student's own nodes.  For a
# headless bridge (CI, a server) build with
#   --build-arg BASE_IMAGE=ros:jazzy-ros-base --build-arg WITH_DESKTOP=0
# Humble works too: --build-arg BASE_IMAGE=tiryoh/ros2-desktop-vnc:humble --build-arg ROS_DISTRO=humble
ARG BASE_IMAGE=tiryoh/ros2-desktop-vnc:jazzy
FROM ${BASE_IMAGE}
ARG ROS_DISTRO=jazzy
ARG WITH_DESKTOP=1
# Extra apt packages, e.g. "xvfb ffmpeg" for a headless RViz you can record.
ARG EXTRA_APT=""
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
USER root

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      curl ca-certificates python3-pil python3-pytest python3-colcon-common-extensions \
      ros-${ROS_DISTRO}-robot-state-publisher ros-${ROS_DISTRO}-control-msgs \
      ros-${ROS_DISTRO}-trajectory-msgs ros-${ROS_DISTRO}-tf2-ros ros-${ROS_DISTRO}-xacro \
      ros-${ROS_DISTRO}-ros2-controllers-test-nodes \
 && if [ "${WITH_DESKTOP}" = "1" ]; then apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-rviz2 ros-${ROS_DISTRO}-joint-state-publisher-gui; fi \
 && if [ -n "${EXTRA_APT}" ]; then apt-get install -y --no-install-recommends ${EXTRA_APT}; fi \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/openbeam_ws
COPY src/ src/
RUN src/openbeam_so101_description/fetch_meshes.sh \
 && if [ "${WITH_DESKTOP}" != "1" ]; then \
      sed -i '/<exec_depend>rviz2<\/exec_depend>\|<exec_depend>joint_state_publisher_gui<\/exec_depend>/d' \
        src/openbeam_so101_description/package.xml; fi \
 && source /opt/ros/${ROS_DISTRO}/setup.bash \
 && colcon build --event-handlers console_direct- \
 && python3 -m pytest -q src/openbeam_bridge/test \
 && rm -rf build log

COPY docker/entrypoint.sh /opt/openbeam_ws/entrypoint.sh
COPY examples/ /opt/openbeam_ws/examples/
RUN chmod +x /opt/openbeam_ws/entrypoint.sh \
 && echo "source /opt/ros/${ROS_DISTRO}/setup.bash && source /opt/openbeam_ws/install/setup.bash" >> /etc/bash.bashrc
