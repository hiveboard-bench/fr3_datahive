#!/usr/bin/env python3
"""Read Linux joydev devices and turn their events into a target pose."""

from __future__ import annotations

import array
import fcntl
import glob
import math
import os
import select
import struct
import threading
import time
from typing import Dict, Iterable, Optional, Tuple

import numpy as np
import tf.transformations as tft

JS_EVENT = struct.Struct("<IhBB")
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
JS_AXIS_MAX = 32767.0
JSIOCGAXES = 0x80016A11
JSIOCGBUTTONS = 0x80016A12
JSIOCGNAME_LEN = 128
JSIOCGNAME = 0x80806A13


def device_name(fd: int) -> str:
    """Name the driver reports for an open joystick node, or "" if it will not say."""
    buffer = array.array("B", [0] * JSIOCGNAME_LEN)
    try:
        fcntl.ioctl(fd, JSIOCGNAME, buffer)
    except OSError:
        return ""
    return buffer.tobytes().split(b"\x00", 1)[0].decode("utf-8", "replace")


def device_counts(fd: int) -> Tuple[Optional[int], Optional[int]]:
    """Axis and button counts for an open joystick node; None where unavailable."""
    counts = []
    for request in (JSIOCGAXES, JSIOCGBUTTONS):
        buffer = array.array("B", [0])
        try:
            fcntl.ioctl(fd, request, buffer)
        except OSError:
            counts.append(None)
        else:
            counts.append(int(buffer[0]))
    return counts[0], counts[1]


def find_device(spec: str, name_match: str = "") -> str:
    """Resolve the joystick node to open, or raise OSError explaining what is missing."""
    if spec and spec != "auto":
        return spec

    candidates = sorted(glob.glob("/dev/input/js*"))
    if not candidates:
        raise OSError(
            "no /dev/input/js* node. Is the pad connected and switched on? A 2.4 GHz "
            "dongle in XInput mode autoloads xpad and appears as js0; also check the "
            "container can see input devices (see the joystick notes in the README)"
        )

    if not name_match:
        return candidates[0]

    wanted = name_match.casefold()
    seen = []
    denied = None
    for path in candidates:
        try:
            fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        except PermissionError as exc:
            denied = denied or exc
            continue
        except OSError:
            continue
        try:
            name = device_name(fd)
        finally:
            os.close(fd)
        seen.append("{} ({})".format(path, name or "unnamed"))
        if wanted in name.casefold():
            return path
    if denied is not None:
        raise denied
    raise OSError(
        "no joystick matching {!r}; found {}".format(
            name_match, ", ".join(seen) or "nothing readable"
        )
    )


def parse_signed_index(spec) -> Tuple[int, float]:
    """Parse an axis reference like "-1" into (index, sign)."""
    if isinstance(spec, bool):
        raise ValueError(f"axis reference {spec!r} is not an axis index")
    if isinstance(spec, int):
        return (abs(spec), -1.0 if spec < 0 else 1.0)

    token = str(spec).strip()
    sign = 1.0
    if token.startswith("-"):
        sign, token = -1.0, token[1:]
    elif token.startswith("+"):
        token = token[1:]
    if not token.isdigit():
        raise ValueError(f"axis reference {spec!r} must be an axis index, optionally signed")
    return int(token), sign


def shape(value: float, deadzone: float, curve: float) -> float:
    """Deadzone and expo curve on a normalised [-1, 1] stick reading."""
    magnitude = min(abs(float(value)), 1.0)
    if magnitude <= deadzone:
        return 0.0
    magnitude = (magnitude - deadzone) / (1.0 - deadzone)
    return math.copysign(magnitude**curve, value)


def clamp_pair(first: float, second: float) -> Tuple[float, float]:
    """Clamp a two-axis stick reading to the unit disc.

    joydev reports the two axes of a stick independently, so a hard diagonal
    push reads (1, 1) -- magnitude 1.41, not 1. Left alone that makes diagonal
    motion 41% faster than the configured speed.
    """
    magnitude = math.hypot(first, second)
    if magnitude <= 1.0:
        return first, second
    return first / magnitude, second / magnitude


def trigger_fraction(value: float, rest: float, deadzone: float = 0.0) -> float:
    """Normalised trigger reading -> [0, 1] press fraction.

    On the XInput layout an analogue trigger rests at -1.0 and reads +1.0 fully
    pressed, so the rest point has to be mapped out before the value means
    anything. `rest` says where that is.
    """
    span = 1.0 - rest
    if span <= 0.0:
        raise ValueError(f"trigger rest value {rest!r} leaves no travel")
    fraction = min(max((float(value) - rest) / span, 0.0), 1.0)
    return 0.0 if fraction <= deadzone else fraction


class JoystickState:
    """Axis and button values as joydev reports them, safe to read from another thread."""

    def __init__(self, trigger_axes: Iterable[int] = (), trigger_rest: float = -1.0) -> None:
        """Start with every trigger axis at its resting value."""
        self._lock = threading.Lock()
        self._axes: Dict[int, float] = {}
        self._buttons: Dict[int, bool] = {}
        self._presses: Dict[int, int] = {}
        self._events = 0
        self._last_event = 0.0

        for index in trigger_axes:
            self._axes[index] = trigger_rest

    def apply(self, ev_type: int, number: int, value: int, now: Optional[float] = None) -> None:
        """Fold one joydev event into the current axis and button state."""
        timestamp = time.monotonic() if now is None else now
        kind = ev_type & ~JS_EVENT_INIT
        with self._lock:
            self._events += 1
            self._last_event = timestamp
            if kind == JS_EVENT_AXIS:
                self._axes[number] = min(max(value / JS_AXIS_MAX, -1.0), 1.0)
            elif kind == JS_EVENT_BUTTON:
                pressed = bool(value)
                if pressed and not self._buttons.get(number, False):
                    self._presses[number] = self._presses.get(number, 0) + 1
                self._buttons[number] = pressed

    def snapshot(self) -> Tuple[Dict[int, float], Dict[int, bool], Dict[int, int], int, float]:
        """One consistent read of everything the control loop needs."""
        with self._lock:
            return (
                dict(self._axes),
                dict(self._buttons),
                dict(self._presses),
                self._events,
                self._last_event,
            )


class JoystickReader:
    """joydev events off /dev/input/js*, in a background thread."""

    def __init__(
        self, path: str, trigger_axes: Iterable[int] = (), trigger_rest: float = -1.0
    ) -> None:
        """Open the device and start the background read thread."""
        self.path = path
        self.state = JoystickState(trigger_axes, trigger_rest)
        self._fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        self.name = device_name(self._fd)
        self.axis_count, self.button_count = device_counts(self._fd)
        self._lost = False
        self._running = True
        self._thread = threading.Thread(target=self._read_loop)
        self._thread.daemon = True
        self._thread.start()

    @property
    def lost(self) -> bool:
        """True once the device node has gone away (dongle pulled, pad unpaired)."""
        return self._lost

    def _read_loop(self) -> None:
        """Read joydev events until closed, flagging the device as lost on error."""
        buffer = b""
        while self._running:
            try:
                ready, _, _ = select.select([self._fd], [], [], 0.5)
            except (OSError, ValueError):
                self._lost = True
                return
            if not ready:
                continue
            try:
                chunk = os.read(self._fd, JS_EVENT.size * 64)
            except BlockingIOError:
                continue
            except OSError:
                self._lost = True
                return
            if not chunk:
                self._lost = True
                return

            buffer += chunk
            while len(buffer) >= JS_EVENT.size:
                _, value, ev_type, number = JS_EVENT.unpack_from(buffer)
                buffer = buffer[JS_EVENT.size :]
                self.state.apply(ev_type, number, value)

    def close(self) -> None:
        """Stop the read thread and close the device."""
        self._running = False
        try:
            os.close(self._fd)
        except OSError:
            pass


class TargetIntegrator:
    """Stick and trigger rates -> the absolute target pose fr3_teleop consumes."""

    def __init__(self, linear_speed: float, vertical_speed: float, angular_rate: float) -> None:
        """Start the virtual target at the origin with no rotation."""
        self.linear_speed = linear_speed
        self.vertical_speed = vertical_speed
        self.angular_rate = angular_rate
        self.position = np.zeros(3)
        self.quat = np.array([0.0, 0.0, 0.0, 1.0])

    def reset(self) -> None:
        """Return the virtual target to the origin with no rotation."""
        self.position = np.zeros(3)
        self.quat = np.array([0.0, 0.0, 0.0, 1.0])

    def integrate(
        self,
        forward: float,
        left: float,
        up: float,
        roll: float,
        pitch: float,
        yaw: float,
        elapsed: float,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Advance the virtual target by `elapsed` seconds of shaped input.

        Rotation is integrated about the *base* axes, so the delta pre-multiplies
        the accumulated quaternion. fr3_teleop anchors this against the arm at
        engage, so only the change from one tick to the next carries meaning.
        """
        self.position = self.position + elapsed * np.array(
            [forward * self.linear_speed, left * self.linear_speed, up * self.vertical_speed]
        )
        omega = self.angular_rate * np.array([roll, pitch, yaw])
        angle = float(np.linalg.norm(omega)) * elapsed
        if angle > 1e-9:
            delta = tft.quaternion_about_axis(angle, omega / np.linalg.norm(omega))
            quat = tft.quaternion_multiply(delta, self.quat)
            self.quat = quat / np.linalg.norm(quat)
        return self.position.copy(), self.quat.copy()
