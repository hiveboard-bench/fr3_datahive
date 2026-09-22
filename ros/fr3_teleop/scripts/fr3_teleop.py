#!/usr/bin/env python3
"""Follow a teleop target pose with the FR3 Cartesian impedance controller."""

from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time
import xml.etree.ElementTree as ElementTree
from typing import Optional, Tuple

import actionlib
import numpy as np
import rospy
import rospkg
import tf.transformations as tft
import yaml
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryResult,
)
from controller_manager_msgs.srv import (
    ListControllers,
    LoadController,
    SwitchController,
    SwitchControllerRequest,
    UnloadController,
)
from fr3_teleop_support.config_loader import load_section
from fr3_teleop_support.joint_limits import JointLimitMonitor, _joint_limits
from franka_gripper.msg import GraspAction, GraspGoal, MoveAction, MoveGoal
from franka_msgs.msg import FrankaState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float32, Float32MultiArray, Float64
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

GRIPPER_GOAL_TIMEOUT = 5.0


def ros_argv(argv):
    """Drop the __name:=/__log:= arguments roslaunch appends to node args."""
    return [a for a in argv if ":=" not in a]


def ee_pose_from_state(o_t_ee) -> Tuple[np.ndarray, np.ndarray]:
    """Position and quaternion from FrankaState.O_T_EE (column major 4x4)."""
    matrix = np.asarray(o_t_ee, dtype=np.float64).reshape(4, 4, order="F")
    return matrix[:3, 3].copy(), tft.quaternion_from_matrix(matrix)


class Fr3Teleop:
    """Turn /teleop/target_pose into equilibrium poses and gripper goals."""

    def __init__(self, args) -> None:
        """Derive the rate limits from args and set up publishers, services, and clients."""
        self.args = args
        self.max_speed = args.max_step * args.rate
        self.max_slew_rate = math.radians(args.max_slew_deg) * args.rate
        self.nominal_period = 1.0 / args.rate
        self.max_period = 2.0 * self.nominal_period
        self._last_tick: Optional[float] = None
        self.fine = False
        self.absolute_target = args.absolute_target
        self.scale = args.scale

        self._lock = threading.Lock()
        self.target: Optional[Tuple[np.ndarray, np.ndarray]] = None
        self.target_time = 0.0
        self.tracking_ok = False
        self.opening: Optional[float] = None
        self.ee: Optional[Tuple[np.ndarray, np.ndarray]] = None
        self.joints: Optional[np.ndarray] = None
        self.ee_time = 0.0
        self.base_yaw = 0.0
        self.base_yaw_anchor = 0.0
        self._home_lock = threading.Lock()
        self._engage_lock = threading.Lock()
        self._engage_generation = 0
        self._home_running = False

        self.engaged = False
        self.clutched = False
        self._auto_engage_armed = args.auto_engage
        self.offset = np.zeros(3)
        self.relative_quat = np.array([0.0, 0.0, 0.0, 1.0])
        self.commanded_pos: Optional[np.ndarray] = None
        self.commanded_quat = np.array([0.0, 0.0, 0.0, 1.0])
        self.published = 0

        self._grip_closed = False
        self._grip_width = args.max_gripper_width
        self._grip_pending = False
        self._grip_sent_time = 0.0

        topic = args.controller.rstrip("/") + "/equilibrium_pose"
        self.pose_pub = rospy.Publisher(topic, PoseStamped, queue_size=1)
        self.engaged_pub = rospy.Publisher("/teleop/engaged", Bool, queue_size=1, latch=True)
        self.grip_width_pub = rospy.Publisher(
            "/teleop/gripper_width_command", Float32, queue_size=1
        )
        self.headroom_pub = rospy.Publisher(
            "/teleop/joint_headroom", Float32MultiArray, queue_size=1
        )
        self.nullspace_client = None
        if args.reanchor_nullspace and not args.dry_run:
            service = args.controller.rstrip("/") + "/set_nullspace_reference"
            self.nullspace_client = rospy.ServiceProxy(service, Trigger)

        self.limits = self._build_limit_monitor()

        rospy.Subscriber("/teleop/target_pose", PoseStamped, self._on_target, queue_size=1)
        rospy.Subscriber("/teleop/gripper_command", Float32, self._on_gripper, queue_size=1)
        rospy.Subscriber("/teleop/tracking_ok", Bool, self._on_tracking, queue_size=1)
        rospy.Subscriber("/teleop/base_yaw", Float64, self._on_base_yaw, queue_size=1)
        rospy.Subscriber(args.franka_state_topic, FrankaState, self._on_state, queue_size=1)

        rospy.Service("/teleop/engage", Trigger, self._on_engage)
        rospy.Service("/teleop/disengage", Trigger, self._on_disengage)
        rospy.Service("/teleop/clutch", SetBool, self._on_clutch)
        rospy.Service("/teleop/toggle_scale", Trigger, self._on_toggle_scale)
        rospy.Service("/teleop/home", Trigger, self._on_home)

        self.grasp_client = None
        self.move_client = None
        if args.gripper and not args.dry_run:
            self.grasp_client = actionlib.SimpleActionClient("/franka_gripper/grasp", GraspAction)
            self.move_client = actionlib.SimpleActionClient("/franka_gripper/move", MoveAction)
            for name, client in (("grasp", self.grasp_client), ("move", self.move_client)):
                if not client.wait_for_server(rospy.Duration(5.0)):
                    rospy.logwarn(
                        "franka_gripper %s action not available; gripper disabled. "
                        "Is franka_control running with load_gripper:=true?",
                        name,
                    )
                    self.grasp_client = self.move_client = None
                    break

        self.engaged_pub.publish(Bool(data=False))
        if args.controller_check:
            self._check_controller()
        self._banner()

    def _build_limit_monitor(self) -> Optional[JointLimitMonitor]:
        """Read the joint travel limits from /robot_description; None if it is unavailable."""
        if self.args.joint_warn_margin <= 0.0:
            return None
        try:
            urdf = rospy.get_param(self.args.robot_description)
        except KeyError:
            rospy.logwarn(
                "%s is not on the parameter server; joint-limit warnings are off",
                self.args.robot_description,
            )
            return None
        suffix = "_link0"
        if not self.args.base_frame.endswith(suffix):
            rospy.logwarn(
                "base_frame %r does not end in %r; joint-limit warnings are off",
                self.args.base_frame,
                suffix,
            )
            return None
        arm_id = self.args.base_frame[: -len(suffix)]
        names = [f"{arm_id}_joint{i}" for i in range(1, 8)]
        try:
            return JointLimitMonitor(
                urdf, names, self.args.joint_warn_margin, self.args.joint_warn_fraction
            )
        except (ElementTree.ParseError, ValueError) as exc:
            rospy.logwarn("joint-limit warnings are off: %s", exc)
            return None

    def _report_headroom(
        self, joints: np.ndarray, velocities: np.ndarray, efforts: np.ndarray
    ) -> None:
        """Publish per-joint travel headroom and warn on any limit the arm is nearing."""
        if self.limits is None:
            return
        room = self.limits.headroom(joints)
        self.headroom_pub.publish(Float32MultiArray(data=[float(v) for v in room]))
        description = self.limits.describe(joints, velocities, efforts)
        if description is not None:
            rospy.logwarn_throttle(2.0, "joint limit approaching: %s", description)

    def _check_controller(self) -> None:
        """Warn early if the impedance controller is not actually running."""
        name = self.args.controller.strip("/")
        try:
            service = "/controller_manager/list_controllers"
            rospy.wait_for_service(service, timeout=5.0)
            controllers = rospy.ServiceProxy(service, ListControllers)().controller
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn(
                "could not query controller_manager (%s); not verifying that %s " "is running",
                exc,
                name,
            )
            return

        running = [c.name for c in controllers if c.state == "running"]
        if name in running:
            rospy.loginfo("controller %s is running", name)
            return
        rospy.logwarn(
            "controller %s is not running (running: %s). Commands will go nowhere "
            "until it is started: rosrun controller_manager controller_manager "
            "start %s",
            name,
            ", ".join(running) or "none",
            name,
        )

    def _banner(self) -> None:
        """Log the topics, limits, and how to engage."""
        args = self.args
        rospy.loginfo(
            "fr3_teleop%s: %s -> %s",
            " [DRY RUN]" if args.dry_run else "",
            "/teleop/target_pose",
            self.pose_pub.resolved_name,
        )
        rospy.loginfo(
            "  scale %.2f coarse / %.2f fine | %.0f Hz | follower tau %.3f s | "
            "max %.2f m/s (%.3f m per command) | slew %.0f deg/s",
            args.scale,
            args.fine_scale,
            args.rate,
            args.follower_time_constant,
            self.max_speed,
            args.max_step,
            math.degrees(self.max_slew_rate),
        )
        rospy.loginfo("  switch ratio with: rosservice call /teleop/toggle_scale")
        rospy.loginfo("  clutch press/release: rosservice call /teleop/clutch true|false")
        rospy.logwarn(
            "  target poses are not filtered by workspace or IK feasibility; "
            "joint-limit monitoring is advisory. The FR3 firmware, impedance "
            "compliance and command-rate limits remain active."
        )
        rospy.loginfo(
            "  orientation %s | gripper %s",
            "follows the target" if args.with_orientation else "locked on engage",
            (
                "off"
                if not args.gripper
                else ("continuous" if args.continuous_gripper else "grasp/release")
            ),
        )
        if args.auto_engage:
            rospy.logwarn(
                "auto-engage is on: the arm starts following as soon as the "
                "input and the robot state are both live"
            )
        else:
            rospy.loginfo("  engage with: rosservice call /teleop/engage")

    def _on_target(self, msg) -> None:
        """Buffer the latest teleop target pose."""
        position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        orientation = np.array(
            [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ]
        )
        with self._lock:
            self.target = (position, orientation)
            self.target_time = time.time()

    def _on_gripper(self, msg) -> None:
        """Buffer the latest gripper opening, clamped to [0, 1]."""
        with self._lock:
            self.opening = float(np.clip(msg.data, 0.0, 1.0))

    def _on_tracking(self, msg) -> None:
        """Buffer whether the tracker still has a good pose."""
        with self._lock:
            self.tracking_ok = bool(msg.data)

    def _on_base_yaw(self, msg) -> None:
        """Buffer the joystick's cumulative whole-arm yaw angle."""
        if not math.isfinite(msg.data):
            rospy.logwarn_throttle(2.0, "ignoring non-finite /teleop/base_yaw")
            return
        with self._lock:
            self.base_yaw = float(msg.data)

    def _on_state(self, msg) -> None:
        """Buffer the arm's end-effector pose and joint positions."""
        pose = ee_pose_from_state(msg.O_T_EE)
        joints = np.asarray(msg.q, dtype=np.float64)
        velocities = np.asarray(msg.dq, dtype=np.float64)
        efforts = np.asarray(msg.tau_J, dtype=np.float64)
        if joints.shape != (7,) or not np.all(np.isfinite(joints)):
            rospy.logwarn_throttle(2.0, "FrankaState contains an invalid seven-joint position")
            joints = None
        if velocities.shape != (7,) or not np.all(np.isfinite(velocities)):
            velocities = None
        if efforts.shape != (7,) or not np.all(np.isfinite(efforts)):
            efforts = None
        with self._lock:
            self.ee = pose
            self.joints = None if joints is None else joints.copy()
            self.ee_time = time.time()
        if joints is not None:
            self._report_headroom(joints, velocities, efforts)

    def _on_engage(self, _request):
        """Handle /teleop/engage and report where the arm anchored."""
        message = self._engage()
        if message is None:
            return TriggerResponse(
                success=True,
                message="engaged at ({:.3f}, {:.3f}, {:.3f})".format(*self.commanded_pos),
            )
        rospy.logwarn("engage refused: %s", message)
        return TriggerResponse(success=False, message=message)

    def _engage(self) -> Optional[str]:
        """Anchor the target to the arm. Returns None on success, else why not.

        The nullspace service is a blocking call with no timeout, so it runs
        between two short holds of `_lock` rather than inside one: holding the
        lock across it stalls the FrankaState callback and every tick.
        """
        with self._engage_lock:
            with self._lock:
                message, _ = self._engage_precheck_locked()
                generation = self._engage_generation
            if message is not None:
                return message

            if self.nullspace_client is not None:
                try:
                    response = self.nullspace_client()
                except rospy.ServiceException as exc:
                    return "could not re-anchor the controller nullspace: {}".format(exc)
                if not response.success:
                    return "controller refused to re-anchor its nullspace: {}".format(
                        response.message or "no reason given"
                    )

            with self._lock:
                if generation != self._engage_generation:
                    return "engage cancelled by a concurrent disengage"
                message, anchor = self._engage_precheck_locked()
                if message is not None:
                    return message
                self._engage_commit_locked(anchor)
        return None

    def _engage_precheck_locked(self):
        """Check that engaging is possible and copy the anchor; (why_not, anchor)."""
        now = time.time()
        if self.ee is None or (now - self.ee_time) > self.args.timeout:
            return (
                f"no recent robot state on {self.args.franka_state_topic} "
                "-- is franka_control running?",
                None,
            )
        if self.target is None or (now - self.target_time) > self.args.timeout:
            return (
                "no recent target on /teleop/target_pose -- is joystick_teleop.py running?",
                None,
            )
        if not self.tracking_ok:
            return ("the input source reports tracking lost (/teleop/tracking_ok is false)", None)

        ee_pos, ee_quat = self.ee
        target_pos, target_quat = self.target
        return (None, (ee_pos.copy(), ee_quat.copy(), target_pos.copy(), target_quat.copy()))

    def _engage_commit_locked(self, anchor) -> None:
        """Store the target contract and mark the node engaged."""
        ee_pos, ee_quat, target_pos, target_quat = anchor
        if not self.absolute_target:
            self.offset = ee_pos - self.scale * target_pos
            self.relative_quat = tft.quaternion_multiply(
                tft.quaternion_inverse(target_quat), ee_quat
            )
        self.commanded_pos = ee_pos.copy()
        self.commanded_quat = ee_quat.copy()
        self.base_yaw_anchor = self.base_yaw
        self._last_tick = None
        self.engaged = True
        self.clutched = False
        self._auto_engage_armed = self.args.auto_engage
        self.engaged_pub.publish(Bool(data=True))

        rospy.loginfo("engaged at (%.3f, %.3f, %.3f)", *ee_pos)

    def _on_toggle_scale(self, _request):
        """Swap between the coarse and fine ratios without moving the arm."""
        with self._lock:
            self.fine = not self.fine
            self.scale = self.args.fine_scale if self.fine else self.args.scale
            if (
                not self.absolute_target
                and self.engaged
                and self.commanded_pos is not None
                and self.target is not None
            ):
                angle = self.base_yaw - self.base_yaw_anchor
                rotation = self._base_rotation(-angle)
                self.offset = rotation.dot(self.commanded_pos) - self.scale * self.target[0]
            scale = self.scale
        message = "{} scale {:.2f} ({:.0f} mm of hand travel per 10 mm of arm)".format(
            "fine" if self.fine else "coarse", scale, 10.0 / scale
        )
        rospy.loginfo(message)
        return TriggerResponse(success=True, message=message)

    def _on_disengage(self, _request):
        """Handle /teleop/disengage, leaving the arm at its last equilibrium pose."""
        with self._lock:
            self.engaged = False
            self.clutched = False
            self._last_tick = None
            self._engage_generation += 1
            self._auto_engage_armed = False
        self.engaged_pub.publish(Bool(data=False))
        if self.args.auto_engage:
            rospy.logwarn(
                "disengaged -- the arm holds its last equilibrium pose; "
                "auto-engage is off until the next /teleop/engage"
            )
        else:
            rospy.logwarn("disengaged -- the arm holds its last equilibrium pose")
        return TriggerResponse(success=True, message="disengaged")

    def _on_clutch(self, request):
        """Hold while pressed, then re-anchor at the held command on release."""
        with self._lock:
            if not self.engaged:
                return SetBoolResponse(success=False, message="teleop is not engaged")
            if request.data:
                if self.clutched:
                    return SetBoolResponse(success=True, message="clutch already pressed")
                self.clutched = True
                self._last_tick = None
                self.engaged_pub.publish(Bool(data=False))
                rospy.loginfo("clutch pressed -- holding equilibrium pose")
                return SetBoolResponse(success=True, message="clutch pressed; pose held")

            if not self.clutched:
                return SetBoolResponse(success=True, message="clutch already released")
            now = time.time()
            if self.target is None or (now - self.target_time) > self.args.timeout:
                return SetBoolResponse(success=False, message="cannot release: no recent target")
            if not self.tracking_ok:
                return SetBoolResponse(success=False, message="cannot release: tracking lost")
            target_pos, target_quat = self.target
            if not self.absolute_target:
                self.offset = self.commanded_pos - self.scale * target_pos
                self.relative_quat = tft.quaternion_multiply(
                    tft.quaternion_inverse(target_quat), self.commanded_quat
                )
                self.base_yaw_anchor = self.base_yaw
            self.clutched = False
            self._last_tick = None
            self.engaged_pub.publish(Bool(data=True))

        rospy.loginfo("clutch released -- re-anchored without moving")
        return SetBoolResponse(success=True, message="clutch released; re-anchored")

    @staticmethod
    def _base_rotation(angle: float) -> np.ndarray:
        """Rotation matrix for a yaw about the robot base Z axis."""
        cosine, sine = math.cos(angle), math.sin(angle)
        return np.array([[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]])

    def _on_home(self, _request):
        """Run Home synchronously through controller-manager and trajectory services."""
        with self._home_lock:
            if self._home_running:
                return TriggerResponse(success=False, message="Home is already running")
            self._home_running = True
        try:
            return self._run_home()
        finally:
            with self._home_lock:
                self._home_running = False

    @staticmethod
    def _controller_name(action_namespace: str) -> str:
        """Derive the trajectory controller name from its action namespace."""
        suffix = "/follow_joint_trajectory"
        if not action_namespace.endswith(suffix):
            raise ValueError("Home action_namespace must end with {}".format(suffix))
        return action_namespace[: -len(suffix)].strip("/")

    def _home_plan(self, start: np.ndarray):
        """Load the shared Home config and build a validated two-point trajectory."""
        package = rospkg.RosPack().get_path("fr3_datahive")
        path = os.path.join(package, "config", "motion.yaml")
        with open(path) as stream:
            config = (yaml.safe_load(stream) or {})["home_position"]

        pose_name = config["default_pose"]
        target = np.asarray(config["poses"][pose_name], dtype=np.float64)
        if start.shape != target.shape:
            raise ValueError(
                "Home target has {} joints, robot state has {}".format(target.size, start.size)
            )

        urdf = rospy.get_param(self.args.robot_description)
        arm_id = self.args.base_frame.rsplit("_link0", 1)[0]
        names = ["{}_joint{}".format(arm_id, i) for i in range(1, target.size + 1)]
        raw_limits = _joint_limits(urdf)
        missing = [name for name in names if name not in raw_limits]
        if missing:
            raise ValueError("URDF has no Home limits for {}".format(", ".join(missing)))
        lower = np.array([raw_limits[name][0] for name in names])
        upper = np.array([raw_limits[name][1] for name in names])
        velocity = np.array([raw_limits[name][2] for name in names])
        margin = float(config["limit_margin"])
        target = np.clip(target, lower + margin, upper - margin)

        speed_fraction = float(config["speed_fraction"])
        seconds = max(
            float(config["minimum_duration"]),
            float(np.max(np.abs(target - start) / (speed_fraction * velocity))),
        )
        settle = float(config["settle_duration"])

        trajectory = JointTrajectory()
        trajectory.joint_names = names
        first = JointTrajectoryPoint()
        first.positions = start.tolist()
        first.velocities = [0.0] * target.size
        first.accelerations = [0.0] * target.size
        first.time_from_start = rospy.Duration(settle)
        last = JointTrajectoryPoint()
        last.positions = target.tolist()
        last.velocities = [0.0] * target.size
        last.accelerations = [0.0] * target.size
        last.time_from_start = rospy.Duration(settle + seconds)
        trajectory.points = [first, last]
        return config, trajectory, seconds

    @staticmethod
    def _call_controller_service(name, service_type, *args) -> bool:
        """Call one controller-manager service and return its `ok` field."""
        rospy.wait_for_service(name, timeout=5.0)
        return bool(rospy.ServiceProxy(name, service_type)(*args).ok)

    @staticmethod
    def _controller_states():
        """Return controller-manager states keyed by controller name."""
        service = "/controller_manager/list_controllers"
        rospy.wait_for_service(service, timeout=5.0)
        controllers = rospy.ServiceProxy(service, ListControllers)().controller
        return {controller.name: controller.state for controller in controllers}

    def _switch_controllers(self, start, stop) -> bool:
        """Atomically exchange controllers under STRICT controller-manager semantics."""
        request = SwitchControllerRequest()
        request.start_controllers = list(start)
        request.stop_controllers = list(stop)
        request.strictness = SwitchControllerRequest.STRICT
        request.start_asap = False
        request.timeout = 0.0
        rospy.wait_for_service("/controller_manager/switch_controller", timeout=5.0)
        response = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)(
            request
        )
        return bool(response.ok)

    def _run_home(self):
        """Disengage, execute Home directly, and always restore impedance control."""
        self._on_disengage(None)
        if self.args.dry_run:
            rospy.logwarn("[dry run] Home requested; no controller was switched")
            return TriggerResponse(success=True, message="dry-run Home complete")

        with self._lock:
            start = None if self.joints is None else self.joints.copy()
        if start is None:
            return TriggerResponse(success=False, message="Home rejected: no valid robot joints")

        try:
            config, trajectory, seconds = self._home_plan(start)
        except (
            KeyError,
            OSError,
            ValueError,
            yaml.YAMLError,
            rospkg.ResourceNotFound,
            rospy.ROSException,
        ) as exc:
            rospy.logerr("HOME rejected: %s", exc)
            return TriggerResponse(
                success=False, message="Home configuration failed: {}".format(exc)
            )

        action_namespace = str(config["action_namespace"])
        position_controller = self._controller_name(action_namespace)
        impedance_controller = self.args.controller.strip("/")
        loaded_here = False
        impedance_stopped = False
        position_started = False
        result_message = "Home failed"
        success = False
        try:
            rospy.logwarn(
                "HOME: moving over %.1f s through an uncollision-checked joint-space path; "
                "keep the workspace clear and the e-stop in hand",
                seconds,
            )
            position_state = self._controller_states().get(position_controller)
            if position_state is None:
                loaded_here = self._call_controller_service(
                    "/controller_manager/load_controller", LoadController, position_controller
                )
                if not loaded_here:
                    raise RuntimeError("could not load {}".format(position_controller))

            if position_state == "running":
                rospy.logwarn(
                    "%s was already running; reusing it and restoring impedance afterward",
                    position_controller,
                )
                impedance_stopped = True
                position_started = True
            else:
                impedance_stopped = self._switch_controllers([], [impedance_controller])
                if not impedance_stopped:
                    raise RuntimeError("could not stop impedance control")
                position_started = self._switch_controllers([position_controller], [])
                if not position_started:
                    raise RuntimeError("could not start trajectory control")

            client = actionlib.SimpleActionClient(action_namespace, FollowJointTrajectoryAction)
            if not client.wait_for_server(rospy.Duration(float(config["action_server_timeout"]))):
                raise RuntimeError("trajectory action server did not appear")
            goal = FollowJointTrajectoryGoal()
            goal.trajectory = trajectory
            client.send_goal(goal)
            timeout = (
                float(config["settle_duration"]) + seconds + float(config["result_timeout_padding"])
            )
            if not client.wait_for_result(rospy.Duration(timeout)):
                client.cancel_goal()
                raise RuntimeError("Home trajectory timed out and was cancelled")
            result = client.get_result()
            if result is None or result.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
                code = None if result is None else result.error_code
                detail = "" if result is None else result.error_string
                raise RuntimeError("Home trajectory failed: {} {}".format(code, detail).strip())
            success = True
            result_message = "Home complete; teleop remains disengaged"
            rospy.loginfo(result_message)
        except (RuntimeError, rospy.ROSException, rospy.ServiceException) as exc:
            result_message = str(exc)
            rospy.logerr("HOME failed: %s", exc)
        finally:
            trajectory_stopped = not position_started
            if position_started:
                try:
                    trajectory_stopped = self._switch_controllers([], [position_controller])
                except (rospy.ROSException, rospy.ServiceException):
                    trajectory_stopped = False
                if not trajectory_stopped:
                    success = False
                    result_message = "trajectory controller did not stop"
                    rospy.logerr("HOME CRITICAL: trajectory controller did not stop")

            if loaded_here and trajectory_stopped:
                try:
                    unloaded = self._call_controller_service(
                        "/controller_manager/unload_controller",
                        UnloadController,
                        position_controller,
                    )
                except (rospy.ROSException, rospy.ServiceException):
                    unloaded = False
                if not unloaded:
                    rospy.logwarn("could not unload %s", position_controller)

            if impedance_stopped and trajectory_stopped:
                try:
                    restored = self._switch_controllers([impedance_controller], [])
                except (rospy.ROSException, rospy.ServiceException):
                    restored = False
                if not restored:
                    success = False
                    result_message = "impedance controller did not restart"
                    rospy.logerr(
                        "HOME CRITICAL: impedance controller did not restart; "
                        "check controller_manager before continuing"
                    )

        return TriggerResponse(success=success, message=result_message)

    def tick(self, _event=None) -> None:
        """Advance one control cycle: rate-limit the target and publish it."""
        now = time.time()

        with self._lock:
            engaged = self.engaged
            armed = self._auto_engage_armed
        if not engaged:
            if not armed:
                return
            message = self._engage()
            if message is not None:
                rospy.logwarn_throttle(2.0, "auto-engage blocked: %s", message)
                return

        with self._lock:
            target = self.target
            target_age = now - self.target_time
            tracking_ok = self.tracking_ok
            opening = self.opening
            clutched = self.clutched
            state_age = now - self.ee_time
            scale = self.scale
            base_yaw_delta = self.base_yaw - self.base_yaw_anchor
            offset = self.offset.copy()
            commanded_pos = self.commanded_pos.copy()
            commanded_quat = self.commanded_quat.copy()

        if clutched:
            self._publish_command(opening)
            return

        elapsed = (
            self.nominal_period
            if self._last_tick is None
            else min(now - self._last_tick, self.max_period)
        )
        self._last_tick = now
        max_step = self.max_speed * elapsed
        max_slew = self.max_slew_rate * elapsed

        if target is None or target_age > self.args.timeout:
            rospy.logwarn_throttle(
                2.0, "no target on /teleop/target_pose for %.2f s; " "holding pose", target_age
            )
            self._publish_command(opening)
            return
        if not tracking_ok:
            rospy.logwarn_throttle(2.0, "input source reports tracking lost; holding pose")
            self._publish_command(opening)
            return
        if state_age > self.args.timeout:
            rospy.logwarn_throttle(2.0, "robot state is stale; holding pose")
            self._publish_command(opening)
            return

        target_pos, target_quat = target
        if self.absolute_target:
            desired = target_pos
        else:
            base_rotation = self._base_rotation(base_yaw_delta)
            desired = base_rotation.dot(offset + scale * target_pos)
        candidate_pos = commanded_pos.copy()
        candidate_quat = commanded_quat.copy()

        follower_alpha = -math.expm1(-elapsed / self.args.follower_time_constant)
        step = follower_alpha * (desired - commanded_pos)
        distance = float(np.linalg.norm(step))
        safety_limited = distance > max_step
        if distance > self.args.threshold:
            if safety_limited:
                step *= max_step / distance
            candidate_pos += step

        if self.args.with_orientation:
            if self.absolute_target:
                desired_quat = target_quat
            else:
                orbit_quat = tft.quaternion_about_axis(base_yaw_delta, (0.0, 0.0, 1.0))
                desired_quat = tft.quaternion_multiply(
                    orbit_quat, tft.quaternion_multiply(target_quat, self.relative_quat)
                )
            candidate_quat = self._slew(commanded_quat, desired_quat, max_slew)

        position_changed = not np.array_equal(candidate_pos, commanded_pos)
        orientation_changed = not np.array_equal(candidate_quat, commanded_quat)
        if position_changed or orientation_changed:
            with self._lock:
                self.commanded_pos = candidate_pos
                self.commanded_quat = candidate_quat
                if safety_limited and not self.absolute_target:
                    self.offset = (
                        self._base_rotation(-base_yaw_delta).dot(self.commanded_pos)
                        - self.scale * target_pos
                    )

        if opening is not None and self.args.gripper:
            self._drive_gripper(opening)
        self._publish_command(opening)

    def _publish_command(self, opening: Optional[float]) -> None:
        """Publish what this node is commanding right now."""
        if self.commanded_pos is None:
            return
        self._publish(self.commanded_pos, self.commanded_quat)
        if self.args.gripper and opening is not None:
            self.grip_width_pub.publish(Float32(data=float(self._grip_width)))

    def _slew(self, current: np.ndarray, target: np.ndarray, max_angle: float) -> np.ndarray:
        """Step `current` toward `target` by at most `max_angle` radians."""
        dot = float(np.clip(abs(np.dot(current, target)), -1.0, 1.0))
        angle = 2.0 * math.acos(dot)
        if angle <= max_angle or angle < 1e-6:
            return target
        return tft.quaternion_slerp(current, target, max_angle / angle)

    def _publish(self, position: np.ndarray, orientation: np.ndarray) -> None:
        """Publish one equilibrium pose, or log it under --dry-run."""
        if self.args.dry_run:
            rospy.loginfo_throttle(0.5, "[dry run] equilibrium (%.3f, %.3f, %.3f)", *position)
            self.published += 1
            return
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.args.base_frame
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(v) for v in position)
        (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ) = (float(v) for v in orientation)
        self.pose_pub.publish(msg)
        self.published += 1

    def _drive_gripper(self, opening: float) -> None:
        """Translate a 0-1 opening into grasp/move goals for the gripper."""
        if self.args.continuous_gripper:
            width = opening * self.args.max_gripper_width
            if abs(width - self._grip_width) > self.args.gripper_deadband:
                self._send_move(width)
            return

        if not self._grip_closed and opening < self.args.close_at:
            self._send_grasp()
        elif self._grip_closed and opening > self.args.open_at:
            self._send_move(self.args.max_gripper_width)

    def _can_send(self) -> bool:
        """True if no goal is in flight and the send interval has elapsed."""
        elapsed = time.time() - self._grip_sent_time
        if self._grip_pending:
            if elapsed < GRIPPER_GOAL_TIMEOUT:
                return False
            rospy.logwarn(
                "gripper goal unanswered for %.1f s; abandoning it. "
                "Is franka_gripper still running?",
                elapsed,
            )
            self._grip_pending = False
        return elapsed >= self.args.gripper_interval

    def _send_grasp(self) -> None:
        """Send a Grasp goal at the configured width and force."""
        if not self._can_send():
            return
        self._grip_closed = True
        self._grip_width = self.args.grasp_width
        if self.args.dry_run or self.grasp_client is None:
            rospy.loginfo(
                "[dry run] grasp %.3f m at %.0f N", self.args.grasp_width, self.args.grasp_force
            )
            return
        goal = GraspGoal()
        goal.width = self.args.grasp_width
        goal.speed = self.args.gripper_speed
        goal.force = self.args.grasp_force
        goal.epsilon.inner = self.args.grasp_epsilon
        goal.epsilon.outer = self.args.grasp_epsilon
        self._send(self.grasp_client, goal)

    def _send_move(self, width: float) -> None:
        """Send a Move goal to `width`, clamped to the gripper's range."""
        if not self._can_send():
            return
        width = float(np.clip(width, 0.0, self.args.max_gripper_width))
        self._grip_closed = False
        self._grip_width = width
        if self.args.dry_run or self.move_client is None:
            rospy.loginfo("[dry run] move to %.3f m", width)
            return
        goal = MoveGoal()
        goal.width = width
        goal.speed = self.args.gripper_speed
        self._send(self.move_client, goal)

    def _send(self, client, goal) -> None:
        """Dispatch a gripper goal and mark one as in flight."""
        self._grip_pending = True
        self._grip_sent_time = time.time()
        client.send_goal(goal, done_cb=self._on_gripper_done)

    def _on_gripper_done(self, _status, _result) -> None:
        """Clear the in-flight flag when a gripper goal completes."""
        self._grip_pending = False

    def status(self, _event=None) -> None:
        """Log a periodic one-line summary of engagement and staleness."""
        now = time.time()
        with self._lock:
            engaged = self.engaged
            clutched = self.clutched
            target_age = now - self.target_time if self.target is not None else float("inf")
            ee = self.ee
            state_age = now - self.ee_time
            opening = self.opening

        if not engaged:
            rospy.loginfo(
                "waiting to engage | target %s | robot state %s",
                "ok" if target_age < self.args.timeout else "missing",
                ("ok" if (ee is not None and state_age < self.args.timeout) else "missing"),
            )
            return

        measured = "({:+.3f}, {:+.3f}, {:+.3f})".format(*ee[0]) if ee is not None else "unknown"
        rospy.loginfo(
            "%s | cmd (%+.3f, %+.3f, %+.3f) | ee %s | %s %.2f | grip %s | sent %d",
            "CLUTCHED" if clutched else "engaged",
            self.commanded_pos[0],
            self.commanded_pos[1],
            self.commanded_pos[2],
            measured,
            "fine" if self.fine else "coarse",
            self.scale,
            "n/a" if opening is None else f"{opening * 100.0:.0f}%",
            self.published,
        )


def main(argv=None) -> int:
    """Parse config and args, then spin the teleop node."""
    raw_argv = ros_argv(sys.argv[1:] if argv is None else argv)
    config_path, defaults = load_section(raw_argv, "controller")
    gripper_defaults = defaults.pop("gripper")
    defaults["controller"] = defaults.pop("namespace")
    defaults.update(
        {
            "max_gripper_width": gripper_defaults["max_width"],
            "close_at": gripper_defaults["close_at"],
            "open_at": gripper_defaults["open_at"],
            "grasp_width": gripper_defaults["grasp_width"],
            "grasp_force": gripper_defaults["grasp_force"],
            "grasp_epsilon": gripper_defaults["grasp_epsilon"],
            "gripper_speed": gripper_defaults["speed"],
            "gripper_interval": gripper_defaults["command_interval"],
            "gripper_deadband": gripper_defaults["deadband"],
        }
    )
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.set_defaults(**defaults)
    parser.add_argument(
        "--config",
        default=config_path,
        help="teleoperation YAML; every numeric/topic setting lives in its " "controller: section",
    )
    parser.add_argument("--controller", help="namespace of the Cartesian impedance controller")
    parser.add_argument("--base-frame", help="frame the equilibrium pose is expressed in")
    parser.add_argument(
        "--follower-time-constant",
        type=float,
        default=defaults.get("follower_time_constant"),
        help="override controller.follower_time_constant for a controlled benchmark sweep",
    )
    parser.add_argument(
        "--with-orientation",
        action="store_true",
        help="let the target rotate the EE (default: lock it on engage)",
    )
    parser.add_argument(
        "--auto-engage",
        action="store_true",
        help="engage as soon as the target and robot state are live",
    )
    parser.add_argument(
        "--absolute-target",
        action="store_true",
        help="interpret /teleop/target_pose directly in base-frame coordinates",
    )
    parser.add_argument(
        "--dry-run", action="store_true", help="log commands instead of sending them"
    )
    parser.add_argument(
        "--no-gripper", dest="gripper", action="store_false", help="leave the gripper alone"
    )
    parser.add_argument(
        "--continuous-gripper",
        action="store_true",
        help="track the commanded opening instead of grasping and releasing",
    )
    parser.add_argument(
        "--no-controller-check",
        dest="controller_check",
        action="store_false",
        help="skip the controller_manager startup check (for native simulation)",
    )
    parser.add_argument(
        "--no-nullspace-reanchor",
        dest="reanchor_nullspace",
        action="store_false",
        help="skip controller nullspace re-anchoring (for native simulation only)",
    )
    args = parser.parse_args(raw_argv)

    if args.open_at <= args.close_at:
        raise SystemExit(
            "controller.gripper.open_at must be above close_at, otherwise the gripper chatters"
        )
    if args.max_step <= 0 or args.rate <= 0:
        raise SystemExit("controller.max_step and controller.rate must be positive")
    if args.follower_time_constant <= 0.0:
        raise SystemExit("controller.follower_time_constant must be positive")
    if args.threshold < 0.0:
        raise SystemExit("controller.threshold must be non-negative")
    if args.scale <= 0 or args.fine_scale <= 0:
        raise SystemExit("controller.scale and controller.fine_scale must be positive")

    rospy.init_node("fr3_teleop", anonymous=True)
    node = Fr3Teleop(args)

    rospy.Timer(rospy.Duration(1.0 / args.rate), node.tick)
    rospy.Timer(rospy.Duration(1.0), node.status)
    rospy.spin()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        pass
