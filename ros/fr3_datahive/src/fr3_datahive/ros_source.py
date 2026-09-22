"""ROS 1 input buffers for the recorder: FrankaState, gripper width, cameras, teleop commands."""

from __future__ import annotations

import threading
import time
from typing import Dict, Optional

import numpy as np
import rospy
from franka_msgs.msg import FrankaState
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float32

from fr3_datahive.convert import ros_image_to_rgb
from fr3_datahive.recorder import Command, Snapshot


class RosSource:
    """Latest-message buffers, read as one consistent Snapshot."""

    def __init__(
        self,
        camera_names,
        camera_topics: Dict[str, str],
        franka_state_topic: str,
        gripper_topic: str,
        teleop_topics: Optional[Dict[str, str]] = None,
    ) -> None:
        """Subscribe to every topic the recording needs."""
        missing = [c for c in camera_names if c not in camera_topics]
        if missing:
            raise SystemExit(
                f"no --camera topic for profile cameras {missing}; expected one NAME:TOPIC per "
                f"entry in {list(camera_names)}"
            )
        self._lock = threading.Lock()
        self._state = None
        self._gripper = 0.0
        self._times: Dict[str, float] = {}
        self._frames: Dict[str, Optional[np.ndarray]] = {c: None for c in camera_names}
        self._command = Command()
        self._teleop = teleop_topics is not None
        self._error: Optional[str] = None

        rospy.Subscriber(franka_state_topic, FrankaState, self._on_state, queue_size=1)
        rospy.Subscriber(gripper_topic, JointState, self._on_gripper, queue_size=1)
        for name in camera_names:
            rospy.Subscriber(
                camera_topics[name], Image, self._on_image, callback_args=name,
                queue_size=1, buff_size=2**24,
            )
        if teleop_topics is not None:
            rospy.Subscriber(teleop_topics["pose"], PoseStamped, self._on_command_pose, queue_size=1)
            rospy.Subscriber(teleop_topics["gripper"], Float32, self._on_command_gripper, queue_size=1)

    def _on_state(self, msg) -> None:
        """Buffer the latest FrankaState."""
        with self._lock:
            self._state = msg
            self._times["franka_state"] = time.time()

    def _on_gripper(self, msg) -> None:
        """Buffer the latest total finger opening in metres."""
        with self._lock:
            self._gripper = float(sum(msg.position))
            self._times["gripper"] = time.time()

    def _on_image(self, msg, name) -> None:
        """Decode and buffer one camera frame."""
        try:
            frame = ros_image_to_rgb(msg)
        except ValueError as exc:
            with self._lock:
                self._error = f"camera {name}: {exc}"
            return
        with self._lock:
            self._frames[name] = frame
            self._times[f"camera:{name}"] = time.time()

    def _on_command_pose(self, msg) -> None:
        """Buffer the latest commanded pose as [x, y, z, qx, qy, qz, qw]."""
        p, o = msg.pose.position, msg.pose.orientation
        with self._lock:
            self._command.pose = np.array([p.x, p.y, p.z, o.x, o.y, o.z, o.w], dtype=np.float64)
            self._command.pose_time = time.time()

    def _on_command_gripper(self, msg) -> None:
        """Buffer the latest commanded gripper width."""
        with self._lock:
            self._command.gripper_width = float(msg.data)
            self._command.gripper_time = time.time()

    def _missing(self) -> list:
        """Names of the inputs that have not delivered yet."""
        with self._lock:
            missing = []
            if self._state is None:
                missing.append("franka_state")
            if "gripper" not in self._times:
                missing.append("gripper")
            missing += [f"camera:{n}" for n, f in self._frames.items() if f is None]
            if self._teleop and (self._command.pose is None or self._command.gripper_width is None):
                missing.append("teleop command")
            return missing

    def wait_ready(self, timeout: float) -> None:
        """Block until every input has delivered, or exit naming what is missing."""
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            if self._error:
                raise SystemExit(self._error)
            if not self._missing():
                return
            time.sleep(0.1)
        raise SystemExit(
            f"timed out after {timeout:.0f}s waiting for: {', '.join(self._missing())}\n"
            "check `rostopic list`, that `make bringup` is running, and the --camera topics"
        )

    def snapshot(self) -> Snapshot:
        """One consistent read of every input."""
        with self._lock:
            command = Command(
                self._command.pose, self._command.gripper_width,
                self._command.pose_time, self._command.gripper_time,
            )
            return Snapshot(self._state, self._gripper, dict(self._frames), command, dict(self._times))
