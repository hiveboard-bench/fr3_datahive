"""ROS-free conversions from FrankaState / Image messages to DataHive arrays."""

from __future__ import annotations

import math
from typing import Any, Callable, Dict

import numpy as np

PROVENANCE = {
    "joint_position": "measured",
    "joint_velocity": "estimated",
    "joint_torque_or_current": "measured",
    "ee_pose": "estimated",
    "ee_state": "measured",
}


def pose_from_column_major(matrix) -> np.ndarray:
    """Convert a column-major 4x4 transform into [x, y, z, qx, qy, qz, qw]."""
    t = np.asarray(matrix, dtype=np.float64).reshape(4, 4, order="F")
    r = t[:3, :3]
    trace = float(np.trace(r))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw, qx, qy, qz = 0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        qw, qx, qy, qz = (r[2, 1] - r[1, 2]) / s, 0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        qw, qx, qy, qz = (r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        qw, qx, qy, qz = (r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s
    return np.concatenate([t[:3, 3], [qx, qy, qz, qw]])


def image_to_rgb(height: int, width: int, encoding: str, step: int, data: bytes) -> np.ndarray:
    """Decode a sensor_msgs/Image payload into an HxWx3 uint8 RGB array without cv_bridge."""
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}.get(encoding)
    if channels is None:
        raise ValueError(f"unsupported image encoding {encoding!r} (use rgb8, bgr8, rgba8, bgra8 or mono8)")
    rows = np.frombuffer(data, dtype=np.uint8).reshape(height, step)
    img = rows[:, : width * channels].reshape(height, width, channels)
    if encoding == "rgb8":
        return img.copy()
    if encoding == "bgr8":
        return img[:, :, ::-1].copy()
    if encoding == "rgba8":
        return img[:, :, :3].copy()
    if encoding == "bgra8":
        return img[:, :, 2::-1].copy()
    return np.repeat(img, 3, axis=2)


def ros_image_to_rgb(msg: Any) -> np.ndarray:
    """Decode a sensor_msgs/Image into an RGB array."""
    return image_to_rgb(msg.height, msg.width, msg.encoding, msg.step, bytes(msg.data))


STATE_EXTRACTORS: Dict[str, Callable[[Any, float], np.ndarray]] = {
    "joint_position": lambda s, g: np.asarray(s.q, dtype=np.float64),
    "joint_velocity": lambda s, g: np.asarray(s.dq, dtype=np.float64),
    "joint_torque_or_current": lambda s, g: np.asarray(s.tau_J, dtype=np.float64),
    "ee_pose": lambda s, g: pose_from_column_major(s.O_T_EE),
    "ee_state": lambda s, g: np.array([g], dtype=np.float64),
}

ACTION_EXTRACTORS: Dict[str, Callable[[Any, float, Any], np.ndarray]] = {
    "joint_position": lambda s, g, c: np.asarray(s.q_d, dtype=np.float64),
    "joint_velocity": lambda s, g, c: np.asarray(s.dq_d, dtype=np.float64),
    "cartesian_position": lambda s, g, c: pose_from_column_major(s.O_T_EE_d),
    "gripper_position": lambda s, g, c: np.array([g], dtype=np.float64),
}

TELEOP_ACTION_EXTRACTORS: Dict[str, Callable[[Any, float, Any], np.ndarray]] = {
    "cartesian_position": lambda s, g, c: c.pose,
    "gripper_position": lambda s, g, c: np.array([c.gripper_width], dtype=np.float64),
}
