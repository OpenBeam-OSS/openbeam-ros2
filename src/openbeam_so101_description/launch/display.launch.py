"""RViz + robot_state_publisher for the SO-101 description.

  ros2 launch openbeam_so101_description display.launch.py            # follow /joint_states (the bridge)
  ros2 launch openbeam_so101_description display.launch.py gui:=true  # sliders, no simulator
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("openbeam_so101_description")
    with open(os.path.join(share, "urdf", "so101.urdf"), encoding="utf-8") as handle:
        urdf = handle.read()
    gui = LaunchConfiguration("gui")
    rsp = LaunchConfiguration("rsp")
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false", description="joint_state_publisher_gui sliders"),
        DeclareLaunchArgument("rsp", default_value="true", description="start robot_state_publisher here"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(package="robot_state_publisher", executable="robot_state_publisher", condition=IfCondition(rsp),
             parameters=[{"robot_description": urdf, "use_sim_time": use_sim_time}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui", condition=IfCondition(gui)),
        Node(package="tf2_ros", executable="static_transform_publisher", condition=IfCondition(gui),
             arguments=["--frame-id", "world", "--child-frame-id", "base_link"]),
        Node(package="rviz2", executable="rviz2", arguments=["-d", os.path.join(share, "rviz", "so101.rviz")],
             parameters=[{"use_sim_time": use_sim_time}]),
    ])
