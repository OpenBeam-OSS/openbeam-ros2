#!/bin/bash
# The action interface MoveIt uses, from the command line.
ros2 action send_goal --feedback /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{trajectory: {
    joint_names: [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll],
    points: [
      {positions: [0.6, -0.4, 0.5, 0.3, 0.0], time_from_start: {sec: 2}},
      {positions: [-0.6, -0.4, 0.5, 0.3, 1.0], time_from_start: {sec: 4}},
      {positions: [0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 6}}]}}"
