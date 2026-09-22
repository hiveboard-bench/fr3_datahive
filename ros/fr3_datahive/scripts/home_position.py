#!/usr/bin/env python3
"""Move the FR3 to a named pose from motion.yaml via FollowJointTrajectory."""

from __future__ import annotations

import os
import sys

import actionlib
import rosgraph
import rospkg
import rospy
import yaml
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryResult,
)
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def load_section(section):
    """Read one section of motion.yaml; ~config overrides the packaged copy."""
    default_path = os.path.join(rospkg.RosPack().get_path("fr3_datahive"), "config", "motion.yaml")
    path = rospy.get_param("~config", default_path)
    with open(path) as stream:
        document = yaml.safe_load(stream) or {}
    values = document.get(section)
    if not isinstance(values, dict):
        raise ValueError(f"motion config {path!r} needs a {section!r} mapping")
    return values


ERROR_NAMES = {
    FollowJointTrajectoryResult.SUCCESSFUL: "SUCCESSFUL",
    FollowJointTrajectoryResult.INVALID_GOAL: "INVALID_GOAL",
    FollowJointTrajectoryResult.INVALID_JOINTS: "INVALID_JOINTS",
    FollowJointTrajectoryResult.OLD_HEADER_TIMESTAMP: "OLD_HEADER_TIMESTAMP",
    FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED: "PATH_TOLERANCE_VIOLATED",
    FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED: "GOAL_TOLERANCE_VIOLATED",
}


def load_limits(joints):
    """FR3 limits straight from franka_description, so there is one source of truth."""
    path = os.path.join(
        rospkg.RosPack().get_path("franka_description"), "robots", "fr3", "joint_limits.yaml"
    )
    with open(path) as f:
        raw = yaml.safe_load(f)
    return {n: raw["joint%d" % n]["limit"] for n in joints}


def main():
    """Drive the arm to the configured pose and return 0 once the goal succeeds."""
    if not rosgraph.is_master_online():
        print("No ROS master. Start franka_control first.")
        return 1

    rospy.init_node("home_position", anonymous=True)

    config = load_section("home_position")
    joints = range(1, int(config["joint_count"]) + 1)
    poses = {
        name: {n: float(value[n - 1]) for n in joints} for name, value in config["poses"].items()
    }
    action_ns = config["action_namespace"]
    joint_state_topic = config["joint_state_topic"]
    state_timeout = float(config["state_timeout"])
    action_server_timeout = float(config["action_server_timeout"])
    result_timeout_padding = float(config["result_timeout_padding"])
    limit_margin = float(config["limit_margin"])
    speed_fraction = float(config["speed_fraction"])
    minimum_duration = float(config["minimum_duration"])
    settle_duration = float(config["settle_duration"])

    pose_name = str(rospy.get_param("~pose", config["default_pose"]))
    if pose_name not in poses:
        rospy.logerr("pose must be one of %s, got %r", sorted(poses), pose_name)
        return 1
    duration_override = float(rospy.get_param("~duration", 0.0))

    try:
        msg = rospy.wait_for_message(joint_state_topic, JointState, timeout=state_timeout)
    except rospy.ROSException:
        rospy.logerr("No /joint_states. Is franka_control running?")
        return 1

    arm_joints = [
        n for n in msg.name if "joint" in n and ("fr3_" in n or "panda_" in n) and "finger" not in n
    ]
    if not arm_joints:
        rospy.logerr("No arm joints in /joint_states.")
        return 1

    if any(n.startswith("panda_") for n in arm_joints):
        rospy.logerr(
            "Joints are named panda_* -- franka_control was launched without "
            "robot:=fr3 and is enforcing Panda joint limits. Relaunch it with "
            "robot:=fr3 before commanding any motion."
        )
        return 1

    limits = load_limits(joints)
    start = {}
    for n in joints:
        name = "fr3_joint%d" % n
        if name not in msg.name:
            rospy.logerr("%s is missing from /joint_states.", name)
            return 1
        start[n] = msg.position[msg.name.index(name)]

    target, clamped = {}, []
    for n in joints:
        lim = limits[n]
        want = poses[pose_name][n]
        got = min(max(want, lim["lower"] + limit_margin), lim["upper"] - limit_margin)
        if abs(got - want) > 1e-9:
            clamped.append((n, want, got))
        target[n] = got

    span = max(
        (abs(target[n] - start[n]) / (speed_fraction * limits[n]["velocity"]) for n in joints),
        default=0.0,
    )
    seconds = duration_override or max(minimum_duration, span)

    print("\n== go to '%s' over %.1fs ==" % (pose_name, seconds))
    if clamped:
        print("  outside the FR3 limits, clamped to %.2f rad inside them:" % limit_margin)
        for n, want, got in clamped:
            print(
                "    joint%d  %+.4f -> %+.4f   (limits [%+.4f,%+.4f])"
                % (n, want, got, limits[n]["lower"], limits[n]["upper"])
            )
    print("  %-10s %9s %9s %9s %9s" % ("joint", "current", "target", "delta", "rad/s"))
    for n in joints:
        delta = target[n] - start[n]
        print(
            "  %-10s %+9.4f %+9.4f %+9.4f %9.3f  (max %.2f)"
            % (
                "joint%d" % n,
                start[n],
                target[n],
                delta,
                abs(delta) / seconds,
                limits[n]["velocity"],
            )
        )

    for n in joints:
        if abs(target[n] - start[n]) / seconds > limits[n]["velocity"]:
            rospy.logerr("joint%d would exceed its velocity limit; raise ~duration.", n)
            return 1

    if max(abs(target[n] - start[n]) for n in joints) < 1e-3:
        print("\nAlready at '%s'; nothing to do." % pose_name)
        return 0

    client = actionlib.SimpleActionClient(action_ns, FollowJointTrajectoryAction)
    rospy.loginfo("Waiting for %s ...", action_ns)
    if not client.wait_for_server(rospy.Duration(action_server_timeout)):
        rospy.logerr(
            "No action server at %s. Did position_joint_trajectory_controller "
            "spawn? Check `rosrun controller_manager controller_manager list`.",
            action_ns,
        )
        return 1

    traj = JointTrajectory()
    traj.joint_names = ["fr3_joint%d" % n for n in joints]

    p0 = JointTrajectoryPoint()
    p0.positions = [start[n] for n in joints]
    p0.velocities = [0.0] * len(joints)
    p0.accelerations = [0.0] * len(joints)
    p0.time_from_start = rospy.Duration(settle_duration)

    p1 = JointTrajectoryPoint()
    p1.positions = [target[n] for n in joints]
    p1.velocities = [0.0] * len(joints)
    p1.accelerations = [0.0] * len(joints)
    p1.time_from_start = rospy.Duration(settle_duration + seconds)

    traj.points = [p0, p1]

    print(
        "\nThe path is a straight joint-space interpolation and is NOT collision "
        "checked.\nClear the workspace and keep the e-stop in hand."
    )
    if not rospy.get_param("~confirm", True):
        rospy.loginfo("~confirm is false; moving without the [ENTER] gate.")
    else:
        try:
            input("-> Press [ENTER] to move (or Ctrl+C to exit)... ")
        except KeyboardInterrupt:
            print("\nExiting...")
            return 0
        except EOFError:
            rospy.logerr(
                "stdin is closed, so the move cannot be confirmed and the arm has "
                "NOT moved. Run `make home` from a terminal, or pass confirm:=false "
                "if the operator already confirmed elsewhere."
            )
            return 1

    traj.header.stamp = rospy.Time(0)
    goal = FollowJointTrajectoryGoal()
    goal.trajectory = traj

    rospy.loginfo("Moving...")
    client.send_goal(goal)
    try:
        finished = client.wait_for_result(
            rospy.Duration(settle_duration + seconds + result_timeout_padding)
        )
    except KeyboardInterrupt:
        client.cancel_goal()
        rospy.logwarn("Interrupted; goal cancelled. The arm stops where it is.")
        return 1
    if not finished:
        client.cancel_goal()
        rospy.logerr("Timed out waiting for the trajectory to finish; goal cancelled.")
        return 1

    result = client.get_result()
    code = result.error_code
    if code != FollowJointTrajectoryResult.SUCCESSFUL:
        rospy.logerr(
            "Trajectory FAILED: %s (%d) %s",
            ERROR_NAMES.get(code, "UNKNOWN"),
            code,
            result.error_string,
        )
        return 1

    rospy.loginfo("At '%s'.", pose_name)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        sys.exit(1)
