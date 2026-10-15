"""The OpenBeam bridge plus robot_state_publisher (and RViz on request).

  ros2 launch openbeam_bridge bridge.launch.py                     # a game, from OPENBEAM_* env
  ros2 launch openbeam_bridge bridge.launch.py backend:=fake       # no game
  ros2 launch openbeam_bridge bridge.launch.py rviz:=true camera_rate_hz:=5
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description = get_package_share_directory("openbeam_so101_description")
    config = os.path.join(get_package_share_directory("openbeam_bridge"), "config", "bridge.yaml")
    with open(os.path.join(description, "urdf", "so101.urdf"), encoding="utf-8") as handle:
        urdf = handle.read()
    args = {
        "backend": "openbeam", "robot": "", "state_rate_hz": "60.0", "command_rate_hz": "60.0",
        "camera_rate_hz": "2.0", "camera_scale": "0.5", "rviz": "false",
        "joint_map": "", "fixed_joints": "", "state_inflight": "1",
    }
    declared = [DeclareLaunchArgument(name, default_value=value) for name, value in args.items()]
    overrides = {name: LaunchConfiguration(name) for name in args if name != "rviz"}
    for name in ("state_rate_hz", "command_rate_hz", "camera_rate_hz", "camera_scale"):
        overrides[name] = ParameterValue(LaunchConfiguration(name), value_type=float)
    overrides["state_inflight"] = ParameterValue(LaunchConfiguration("state_inflight"), value_type=int)
    for name in ("robot", "joint_map", "fixed_joints"):
        overrides[name] = ParameterValue(LaunchConfiguration(name), value_type=str)
    return LaunchDescription(declared + [
        Node(package="openbeam_bridge", executable="bridge", name="openbeam_bridge", output="screen",
             parameters=[config, overrides]),
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
             parameters=[{"robot_description": urdf, "use_sim_time": True}]),
        Node(package="rviz2", executable="rviz2", condition=IfCondition(LaunchConfiguration("rviz")),
             arguments=["-d", os.path.join(description, "rviz", "so101.rviz")],
             parameters=[{"use_sim_time": True}]),
    ])
