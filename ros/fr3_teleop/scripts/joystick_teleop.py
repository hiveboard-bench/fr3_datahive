#!/usr/bin/env python3
"""Gamepad -> /teleop/target_pose + /teleop/gripper_command."""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Optional

import rospy
from fr3_teleop_support.config_loader import load_section
from fr3_teleop_support.joystick import (
    JoystickReader,
    TargetIntegrator,
    clamp_pair,
    find_device,
    parse_signed_index,
    shape,
    trigger_fraction,
)
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float32, Float64
from std_srvs.srv import Trigger


def ros_argv(argv):
    """Drop the __name:=/__log:= arguments roslaunch appends to node args."""
    return [a for a in argv if ":=" not in a]


class JoystickTeleop:
    """Integrate joystick input into the target pose fr3_teleop follows."""

    def __init__(self, args) -> None:
        """Resolve the axis mapping, open the pad, and set up publishers."""
        self.args = args
        self.x_axis, self.x_sign = parse_signed_index(args.axis_x)
        self.y_axis, self.y_sign = parse_signed_index(args.axis_y)
        self.z_axis, self.z_sign = parse_signed_index(args.axis_z)
        self.roll_axis, self.roll_sign = parse_signed_index(args.axis_roll)
        self.pitch_axis, self.pitch_sign = parse_signed_index(args.axis_pitch)
        self.yaw_axis, self.yaw_sign = parse_signed_index(args.axis_yaw)

        self.integrator = TargetIntegrator(
            args.linear_speed, args.vertical_speed, math.radians(args.angular_speed_deg)
        )
        self.opening = 1.0
        self.seen_presses: dict = {}
        self.nominal_period = 1.0 / args.rate
        self.max_period = 2.0 * self.nominal_period
        self._last_tick: Optional[float] = None
        self.base_yaw = 0.0
        self.yaw_mode = "gripper"
        self._home_pressed_since: Optional[float] = None
        self._home_sent = False

        path = find_device(args.device, args.name_match)
        self.reader = JoystickReader(
            path, trigger_axes=(self.z_axis, self.yaw_axis), trigger_rest=args.trigger_rest
        )

        self.pose_pub = rospy.Publisher("/teleop/target_pose", PoseStamped, queue_size=1)
        self.grip_pub = rospy.Publisher("/teleop/gripper_command", Float32, queue_size=1)
        self.ok_pub = rospy.Publisher("/teleop/tracking_ok", Bool, queue_size=1)
        self.base_yaw_pub = rospy.Publisher("/teleop/base_yaw", Float64, queue_size=1)
        self.record_pub = rospy.Publisher("/teleop/record_trigger", Bool, queue_size=1)

        self.engaged = False
        self.engaged_sub = rospy.Subscriber("/teleop/engaged", Bool, self._engaged_cb, queue_size=1)

        self.engage_srv = rospy.ServiceProxy("/teleop/engage", Trigger)
        self.disengage_srv = rospy.ServiceProxy("/teleop/disengage", Trigger)
        self.toggle_scale_srv = rospy.ServiceProxy("/teleop/toggle_scale", Trigger)
        self.home_srv = rospy.ServiceProxy("/teleop/home", Trigger)

        self._banner()

    def _engaged_cb(self, msg: Bool) -> None:
        self.engaged = bool(msg.data)

    def _banner(self) -> None:
        """Log a compact device summary and the safety-critical controls."""
        args = self.args
        reader = self.reader
        rospy.loginfo(
            "joystick_teleop: %s (%s), %s axes, %s buttons",
            reader.path,
            reader.name or "unnamed device",
            "?" if reader.axis_count is None else reader.axis_count,
            "?" if reader.button_count is None else reader.button_count,
        )
        rospy.loginfo(
            "controls: engage=%s disengage=%s record=%s home=hold %s for %.1fs",
            args.engage_button,
            args.disengage_button,
            getattr(args, "record_button", None),
            args.home_button,
            args.home_hold_seconds,
        )

        for name, index in (
            ("axis_x", self.x_axis),
            ("axis_y", self.y_axis),
            ("axis_z", self.z_axis),
            ("axis_roll", self.roll_axis),
            ("axis_pitch", self.pitch_axis),
            ("axis_yaw", self.yaw_axis),
        ):
            if reader.axis_count is not None and index >= reader.axis_count:
                rospy.logwarn(
                    "joystick.%s refers to axis %d but the device reports only "
                    "%d axes. Is the pad in a different input mode? Run with "
                    "--probe to see the live layout.",
                    name,
                    index,
                    reader.axis_count,
                )
        for name, index in (
            ("close_button", args.close_button),
            ("open_button", args.open_button),
            ("z_up_button", args.z_up_button),
            ("yaw_ccw_button", args.yaw_ccw_button),
            ("yaw_mode_button", args.yaw_mode_button),
            ("engage_button", args.engage_button),
            ("disengage_button", args.disengage_button),
            ("scale_button", args.scale_button),
            ("record_button", getattr(args, "record_button", None)),
            ("deadman_button", args.deadman_button),
        ):
            if (
                index is not None
                and reader.button_count is not None
                and index >= reader.button_count
            ):
                rospy.logwarn(
                    "joystick.%s refers to button %d but the device reports only "
                    "%d buttons. Run with --probe to see the live layout.",
                    name,
                    index,
                    reader.button_count,
                )

    def _stick(self, axes: dict, index: int, sign: float) -> float:
        """One shaped stick axis in [-1, 1]."""
        return sign * shape(axes.get(index, 0.0), self.args.deadzone, self.args.curve)

    def _trigger(self, axes: dict, buttons: dict, index: int, sign: float, button) -> float:
        """One axis in [-1, 1] from a trigger one way and a button the other.

        The trigger is proportional; the button is a switch, so it commands the
        full configured rate for as long as it is held. Holding both cancels.
        """
        value = sign * trigger_fraction(
            axes.get(index, self.args.trigger_rest),
            self.args.trigger_rest,
            self.args.trigger_deadzone,
        )
        if button is not None and buttons.get(button, False):
            value -= sign
        return max(min(value, 1.0), -1.0)

    def _tapped(self, presses: dict, button) -> bool:
        """True once per fresh press of `button`, ignoring how long it is held."""
        if button is None:
            return False
        count = presses.get(button, 0)
        if count == self.seen_presses.get(button, 0):
            return False
        self.seen_presses[button] = count
        return True

    def _call(self, name: str, service) -> None:
        """Call one of fr3_teleop's Trigger services, reporting what it said."""
        try:
            response = service()
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn("%s failed (%s); is fr3_teleop running?", name, exc)
            return
        rospy.loginfo("%s: %s", name, response.message)

    def _check_home_hold(self, buttons: dict, now: float) -> None:
        """Request Home once after its button has been held continuously."""
        button = self.args.home_button
        if button is None or not buttons.get(button, False):
            self._home_pressed_since = None
            self._home_sent = False
            return
        if self._home_pressed_since is None:
            self._home_pressed_since = now
            rospy.logwarn(
                "home button held: keep holding for %.1f s to run the uncollision-checked "
                "joint-space move",
                self.args.home_hold_seconds,
            )
            return
        held = now - self._home_pressed_since
        if self._home_sent or held < self.args.home_hold_seconds:
            return
        self._home_sent = True
        rospy.logwarn("home hold confirmed; disengaging teleop and requesting Home")
        self._call("home", self.home_srv)

    def tick(self, _event=None) -> None:
        """Advance one cycle: read the pad, integrate it, and publish the target."""
        now = time.monotonic()
        elapsed = (
            self.nominal_period
            if self._last_tick is None
            else min(now - self._last_tick, self.max_period)
        )
        self._last_tick = now

        if self.reader.lost:
            self.ok_pub.publish(Bool(data=False))
            rospy.logerr_throttle(
                2.0,
                "joystick %s disappeared; holding pose. Reconnect the pad and "
                "restart this node.",
                self.reader.path,
            )
            return

        axes, buttons, presses, events, last_event = self.reader.state.snapshot()
        self._check_home_hold(buttons, now)
        for button, opening in ((self.args.close_button, 0.0), (self.args.open_button, 1.0)):
            if self._tapped(presses, button):
                self.opening = opening
        self.grip_pub.publish(Float32(data=float(self.opening)))

        if self._tapped(presses, self.args.engage_button):
            if (
                self.args.disengage_button is None
                or self.args.disengage_button == self.args.engage_button
            ):
                if self.engaged:
                    self._call("disengage", self.disengage_srv)
                else:
                    self._call("engage", self.engage_srv)
            else:
                self._call("engage", self.engage_srv)
        if (
            self.args.disengage_button is not None
            and self.args.disengage_button != self.args.engage_button
            and self._tapped(presses, self.args.disengage_button)
        ):
            self._call("disengage", self.disengage_srv)
        if self._tapped(presses, self.args.scale_button):
            self._call("toggle_scale", self.toggle_scale_srv)
        if getattr(self.args, "record_button", None) is not None and self._tapped(
            presses, self.args.record_button
        ):
            self.record_pub.publish(Bool(data=True))
            rospy.loginfo("joystick: record trigger tapped (button %s)", self.args.record_button)
        if self._tapped(presses, self.args.yaw_mode_button):
            self.yaw_mode = "base" if self.yaw_mode == "gripper" else "gripper"
            rospy.logwarn("yaw mode: %s", self.yaw_mode.upper())

        moving = True
        if self.args.deadman_button is not None and not buttons.get(
            self.args.deadman_button, False
        ):
            moving = False
        if (
            self.args.input_timeout > 0.0
            and events
            and (time.monotonic() - last_event) > self.args.input_timeout
        ):
            rospy.logwarn_throttle(
                2.0,
                "no joystick event for %.1f s; holding the target. The pad only "
                "reports changes, so a perfectly steady stick can trip this -- "
                "raise joystick.input_timeout or set it to 0 to disable.",
                self.args.input_timeout,
            )
            moving = False

        if moving:
            forward, left = clamp_pair(
                self._stick(axes, self.x_axis, self.x_sign),
                self._stick(axes, self.y_axis, self.y_sign),
            )
            roll, pitch = clamp_pair(
                self._stick(axes, self.roll_axis, self.roll_sign),
                self._stick(axes, self.pitch_axis, self.pitch_sign),
            )
            up = self._trigger(axes, buttons, self.z_axis, self.z_sign, self.args.z_up_button)
            yaw = self._trigger(
                axes, buttons, self.yaw_axis, self.yaw_sign, self.args.yaw_ccw_button
            )
        else:
            forward = left = up = roll = pitch = yaw = 0.0

        gripper_yaw = yaw if self.yaw_mode == "gripper" else 0.0
        position, orientation = self.integrator.integrate(
            forward, left, up, roll, pitch, gripper_yaw, elapsed
        )
        if self.yaw_mode == "base":
            self.base_yaw += yaw * self.integrator.angular_rate * elapsed
        self.base_yaw_pub.publish(Float64(data=self.base_yaw))

        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.args.frame_id
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(v) for v in position)
        (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ) = (float(v) for v in orientation)
        self.pose_pub.publish(msg)
        self.ok_pub.publish(Bool(data=True))

    def status(self, _event=None) -> None:
        """Warn while the pad has sent no events; stay quiet during normal use."""
        _, _, _, events, _ = self.reader.state.snapshot()
        if not events:
            rospy.logwarn_throttle(
                5.0,
                "no events from %s yet. Is the pad switched on and still paired? "
                "Press a button to check.",
                self.reader.path,
            )

    def probe(self, _event=None) -> None:
        """Print the live layout so a remapped or wrong-mode pad can be read off."""
        axes, buttons, presses, events, _ = self.reader.state.snapshot()
        rospy.loginfo("axes %s", " ".join(f"{i}:{axes[i]:+.2f}" for i in sorted(axes)) or "none")
        held = [i for i in sorted(buttons) if buttons[i]]
        rospy.loginfo(
            "buttons held %s | presses %s | %d events",
            held or "none",
            {i: presses[i] for i in sorted(presses)} or "none",
            events,
        )


def main(argv=None) -> int:
    """Parse config and args, then spin the joystick teleop node."""
    raw_argv = ros_argv(sys.argv[1:] if argv is None else argv)
    config_path, defaults = load_section(raw_argv, "joystick")
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.set_defaults(**defaults)
    parser.add_argument(
        "--config",
        default=config_path,
        help="teleoperation YAML; every other setting lives in its " "joystick: section",
    )

    parser.add_argument(
        "--probe",
        action="store_true",
        help="print the live axis/button layout instead of publishing; "
        "use it to remap a pad in a different input mode",
    )
    args = parser.parse_args(raw_argv)

    if args.rate <= 0.0:
        raise SystemExit("joystick.rate must be positive")
    if not 0.0 <= args.deadzone < 1.0:
        raise SystemExit("joystick.deadzone must be in [0, 1)")
    if not 0.0 <= args.trigger_deadzone < 1.0:
        raise SystemExit("joystick.trigger_deadzone must be in [0, 1)")
    if args.curve <= 0.0:
        raise SystemExit("joystick.curve must be positive")
    if args.trigger_rest >= 1.0:
        raise SystemExit("joystick.trigger_rest must be below 1.0, the full-press value")
    if args.linear_speed <= 0.0 or args.vertical_speed <= 0.0:
        raise SystemExit("joystick.linear_speed and joystick.vertical_speed must be positive")
    if args.angular_speed_deg <= 0.0:
        raise SystemExit("joystick.angular_speed_deg must be positive")
    if args.input_timeout < 0.0:
        raise SystemExit("joystick.input_timeout must be non-negative (0 disables it)")
    used: dict = {}
    for name in (
        "close_button",
        "open_button",
        "z_up_button",
        "yaw_ccw_button",
        "yaw_mode_button",
        "engage_button",
        "disengage_button",
        "scale_button",
        "record_button",
        "home_button",
        "deadman_button",
    ):
        index = getattr(args, name)
        if index is None:
            continue
        if index in used:
            raise SystemExit(
                "joystick.{} and joystick.{} are both button {}".format(used[index], name, index)
            )
        used[index] = name
    if args.home_hold_seconds <= 0.0:
        raise SystemExit("joystick.home_hold_seconds must be positive")

    rospy.init_node("joystick_teleop", anonymous=True)
    try:
        node = JoystickTeleop(args)
    except PermissionError as exc:
        raise SystemExit(
            "cannot read {}: {}. Input devices are char major 13 -- the container "
            "needs --device-cgroup-rule='c 13:* rmw' (it is in the Makefile's "
            "RUN_FLAGS). A container started before that rule was added keeps the "
            "old permissions: run `make stop && make run` to recreate it.".format(
                getattr(exc, "filename", args.device), exc
            )
        ) from exc
    except OSError as exc:
        raise SystemExit(f"joystick unavailable: {exc}") from exc

    if args.probe:
        rospy.logwarn("--probe: nothing is published. Move the sticks and press buttons.")
        rospy.Timer(rospy.Duration(0.5), node.probe)
    else:
        rospy.Timer(rospy.Duration(1.0 / args.rate), node.tick)
        rospy.Timer(rospy.Duration(2.0), node.status)
    try:
        rospy.spin()
    finally:
        node.reader.close()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        pass
