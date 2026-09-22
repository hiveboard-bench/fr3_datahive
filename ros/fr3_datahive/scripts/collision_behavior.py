#!/usr/bin/env python3
"""Apply the collision reflex thresholds from motion.yaml to the running FR3."""

from __future__ import annotations

import math
import os
import sys

import rosgraph
import rospkg
import rospy
import yaml
from franka_msgs.srv import SetFullCollisionBehavior

SERVICE = "/franka_control/set_full_collision_behavior"
TORQUE_KEYS = (
    ("lower_torque_thresholds_acceleration", "upper_torque_thresholds_acceleration"),
    ("lower_torque_thresholds_nominal", "upper_torque_thresholds_nominal"),
)
FORCE_KEYS = (
    ("lower_force_thresholds_acceleration", "upper_force_thresholds_acceleration"),
    ("lower_force_thresholds_nominal", "upper_force_thresholds_nominal"),
)


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


def thresholds(config, key, count):
    """Return one threshold array, checked for length and positive finite values."""
    values = [float(value) for value in config[key]]
    if len(values) != count:
        raise ValueError(f"{key} needs {count} values, got {len(values)}")
    if any(not math.isfinite(value) or value <= 0.0 for value in values):
        raise ValueError(f"{key} must be finite and positive, got {values}")
    return values


def build_request(config):
    """Return all eight threshold arrays as service keyword arguments."""
    request = {}
    for pairs, count in ((TORQUE_KEYS, 7), (FORCE_KEYS, 6)):
        for lower_key, upper_key in pairs:
            request[lower_key] = thresholds(config, lower_key, count)
            request[upper_key] = thresholds(config, upper_key, count)
            for index, (lower, upper) in enumerate(zip(request[lower_key], request[upper_key])):
                if lower > upper:
                    raise ValueError(
                        f"{lower_key}[{index}] is {lower}, above {upper_key}[{index}] at {upper}"
                    )
    return request


def main():
    """Raise the reflex thresholds and return 0 once the firmware accepts them."""
    if not rosgraph.is_master_online():
        print("No ROS master. Start franka_control first.")
        return 1

    rospy.init_node("collision_behavior", anonymous=True)

    config = load_section("collision_behavior")
    try:
        request = build_request(config)
    except (KeyError, TypeError, ValueError) as exc:
        rospy.logerr("collision_behavior thresholds are unusable: %s", exc)
        return 1
    service_timeout = float(config["service_timeout"])

    try:
        rospy.wait_for_service(SERVICE, timeout=service_timeout)
    except rospy.ROSException:
        rospy.logerr(
            "No %s after %.1f s, so the arm keeps the franka_ros defaults. "
            "Is franka_control running?",
            SERVICE,
            service_timeout,
        )
        return 1

    print("\n== collision reflex thresholds ==")
    for pairs in (TORQUE_KEYS, FORCE_KEYS):
        for key in (pair_key for pair in pairs for pair_key in pair):
            print("  %-38s %s" % (key, " ".join("%6.1f" % value for value in request[key])))

    try:
        response = rospy.ServiceProxy(SERVICE, SetFullCollisionBehavior)(**request)
    except rospy.ServiceException as exc:
        rospy.logerr("%s failed: %s", SERVICE, exc)
        return 1

    if not response.success:
        rospy.logerr("The firmware refused the thresholds: %s", response.error)
        return 1

    rospy.loginfo("Collision reflex thresholds applied; the upper arrays are what stops the arm.")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        sys.exit(1)
