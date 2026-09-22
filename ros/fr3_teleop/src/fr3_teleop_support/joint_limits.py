#!/usr/bin/env python3
"""Warn when a joint nears its position, velocity or effort limit, without acting on the robot."""

from __future__ import annotations

import xml.etree.ElementTree as ElementTree
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np


def _joint_limits(urdf_xml: str) -> Dict[str, Tuple[float, float, float, float]]:
    """Map joint name to (lower, upper, velocity, effort) for every limited joint."""
    root = ElementTree.fromstring(urdf_xml)
    limits = {}
    for joint in root.findall("joint"):
        element = joint.find("limit")
        name = joint.get("name")
        if element is None or name is None:
            continue
        lower, upper = element.get("lower"), element.get("upper")
        velocity, effort = element.get("velocity"), element.get("effort")
        if None in (lower, upper, velocity, effort):
            continue
        limits[name] = (float(lower), float(upper), float(velocity), float(effort))
    return limits


class JointLimitMonitor:
    """Track how much room each joint has left in position, speed and torque."""

    def __init__(
        self,
        urdf_xml: str,
        joint_names: Sequence[str],
        warn_margin: float,
        warn_fraction: float = 0.8,
    ) -> None:
        """Parse the per-joint limits out of a URDF string."""
        if warn_margin <= 0.0:
            raise ValueError("joint limit warn_margin must be positive")
        if not 0.0 < warn_fraction <= 1.0:
            raise ValueError("joint limit warn_fraction must be in (0, 1]")

        limits = _joint_limits(urdf_xml)
        missing = [name for name in joint_names if name not in limits]
        if missing:
            raise ValueError("URDF has no limits for {}".format(", ".join(missing)))

        self.joint_names = list(joint_names)
        rows = [limits[name] for name in joint_names]
        self.lower = np.array([r[0] for r in rows], dtype=np.float64)
        self.upper = np.array([r[1] for r in rows], dtype=np.float64)
        self.max_velocity = np.array([r[2] for r in rows], dtype=np.float64)
        self.max_effort = np.array([r[3] for r in rows], dtype=np.float64)
        for values, what in ((self.max_velocity, "velocity"), (self.max_effort, "effort")):
            if not np.all(np.isfinite(values)) or np.any(values <= 0.0):
                raise ValueError("URDF {} limits must be finite and positive".format(what))

        self.warn_margin = float(warn_margin)
        self.warn_fraction = float(warn_fraction)

    def _vector(self, values: np.ndarray, what: str) -> np.ndarray:
        """Return values as a float array of one entry per joint."""
        array = np.asarray(values, dtype=np.float64)
        if array.shape != self.lower.shape:
            raise ValueError(
                "{} has shape {}, expected {}".format(what, array.shape, self.lower.shape)
            )
        return array

    def headroom(self, joints: np.ndarray) -> np.ndarray:
        """Signed radians to the nearest travel limit per joint; negative means past it."""
        q = self._vector(joints, "joint positions")
        return np.minimum(q - self.lower, self.upper - q)

    def velocity_load(self, velocities: np.ndarray) -> np.ndarray:
        """Speed as a fraction of each joint's velocity limit."""
        return np.abs(self._vector(velocities, "joint velocities")) / self.max_velocity

    def effort_load(self, efforts: np.ndarray) -> np.ndarray:
        """Torque as a fraction of each joint's effort limit."""
        return np.abs(self._vector(efforts, "joint efforts")) / self.max_effort

    def worst(self, joints: np.ndarray) -> Tuple[int, float]:
        """Return (index, headroom) for the joint closest to a travel limit."""
        room = self.headroom(joints)
        index = int(np.argmin(room))
        return index, float(room[index])

    def describe(
        self,
        joints: np.ndarray,
        velocities: Optional[np.ndarray] = None,
        efforts: Optional[np.ndarray] = None,
    ) -> Optional[str]:
        """One line naming every joint near a limit, or None when all three are clear."""
        parts: List[str] = []
        room = self.headroom(joints)
        for i in np.argsort(room):
            if room[i] >= self.warn_margin:
                break
            near = (
                "lower" if (joints[i] - self.lower[i]) <= (self.upper[i] - joints[i]) else "upper"
            )
            parts.append("{} {:+.3f} rad from {}".format(self.joint_names[i], room[i], near))

        for values, accessor, unit in (
            (velocities, self.velocity_load, "speed"),
            (efforts, self.effort_load, "torque"),
        ):
            if values is None:
                continue
            load = accessor(values)
            for i in np.argsort(-load):
                if load[i] < self.warn_fraction:
                    break
                parts.append(
                    "{} at {:.0f}% {} limit".format(self.joint_names[i], 100.0 * load[i], unit)
                )

        return "; ".join(parts) if parts else None
