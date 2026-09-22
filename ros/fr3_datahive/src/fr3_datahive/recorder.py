"""ROS-free episode recorder: fixed-rate state/command logging plus per-camera video."""

from __future__ import annotations

import shutil
import subprocess
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

import numpy as np

from fr3_datahive.convert import (
    ACTION_EXTRACTORS,
    PROVENANCE,
    STATE_EXTRACTORS,
    TELEOP_ACTION_EXTRACTORS,
)

MIN_EPISODE_S = 1.0
MAX_EPISODE_S = 600.0
MIN_RATE_HZ = 100.0


class EpisodeDiscarded(RuntimeError):
    """The episode must not be kept."""


class StaleSource(EpisodeDiscarded):
    """A camera, the gripper, or franka_control stopped publishing mid-episode."""


class StaleCommand(EpisodeDiscarded):
    """The teleop command stream stopped while the episode was recording."""


class ConstantAction(EpisodeDiscarded):
    """The action never changed, so whatever moved the arm was not the recorded command."""


class TooShort(EpisodeDiscarded):
    """The episode ended before the minimum length."""


@dataclass
class Command:
    """Latest teleop command; empty until the teleop node is engaged."""

    pose: Optional[np.ndarray] = None
    gripper_width: Optional[float] = None
    pose_time: float = 0.0
    gripper_time: float = 0.0

    def age(self, now: float) -> float:
        """Seconds since the older of the two commands last arrived."""
        return now - min(self.pose_time, self.gripper_time)


@dataclass
class Snapshot:
    """One consistent read of every input, with the arrival time of each source."""

    state: Any
    gripper: float
    frames: Dict[str, np.ndarray]
    command: Command
    times: Dict[str, float] = field(default_factory=dict)


def resolve_camera_topics(names: List[str], topics: Dict[str, str]) -> Dict[str, str]:
    """Map each profile camera name to its topic, ignoring case ('Front' matches --camera front)."""
    by_lower = {k.lower(): v for k, v in topics.items()}
    missing = [n for n in names if n.lower() not in by_lower]
    if missing:
        raise SystemExit(
            f"no --camera topic for profile cameras {missing}; have {sorted(topics)}. "
            "Rename the cameras in samples/robot_profile.yaml or add --camera NAME:TOPIC."
        )
    return {n: by_lower[n.lower()] for n in names}


def check_profile_supported(profile, teleop_actions: bool) -> None:
    """Fail at startup, not mid-episode, if the profile asks for something we cannot fill."""
    problems: List[str] = []
    unknown_action = [a for a in profile.action_space if a not in ACTION_EXTRACTORS]
    if unknown_action:
        problems.append(
            f"action_space entries not supported: {unknown_action} (supported: {sorted(ACTION_EXTRACTORS)})"
        )
    if teleop_actions:
        uncommanded = [a for a in profile.action_space if a not in TELEOP_ACTION_EXTRACTORS]
        if uncommanded:
            problems.append(
                f"--teleop-actions cannot supply {uncommanded}; it commands {sorted(TELEOP_ACTION_EXTRACTORS)}"
            )
    if not profile.control_freq or float(profile.control_freq) < MIN_RATE_HZ:
        problems.append(
            f"control_freq {profile.control_freq} Hz is below the {MIN_RATE_HZ:.0f} Hz DataHive minimum"
        )
    if not profile.cameras:
        problems.append("the profile lists no cameras")
    if problems:
        raise SystemExit("\n".join(problems))


class VideoSink:
    """Encodes RGB frames to one mp4, preferring ffmpeg/H.264 and falling back to OpenCV mp4v."""

    def __init__(self, path: Path, width: int, height: int, fps: float) -> None:
        """Open the encoder for `path`."""
        self.path = Path(path)
        self._proc = None
        self._cv = None
        if shutil.which("ffmpeg"):
            self._proc = subprocess.Popen(
                [
                    "ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
                    "-s", f"{width}x{height}", "-r", f"{fps:g}", "-i", "-", "-an",
                    "-vf", "crop=trunc(iw/2)*2:trunc(ih/2)*2",
                    "-c:v", "libx264", "-preset", "veryfast", "-crf", "20",
                    "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(self.path),
                ],
                stdin=subprocess.PIPE,
            )
        else:
            import cv2

            self._cv = cv2.VideoWriter(
                str(self.path), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height)
            )

    def write(self, rgb: np.ndarray) -> None:
        """Append one RGB frame."""
        if self._proc is not None:
            self._proc.stdin.write(np.ascontiguousarray(rgb).tobytes())
        else:
            self._cv.write(np.ascontiguousarray(rgb[:, :, ::-1]))

    def close(self) -> None:
        """Flush and finalise the file."""
        if self._proc is not None:
            self._proc.stdin.close()
            self._proc.wait()
        elif self._cv is not None:
            self._cv.release()


class VideoRecorder:
    """Writes the latest frame of every camera at a fixed rate on a background thread."""

    def __init__(
        self,
        out_dir: Path,
        frames: Callable[[], Dict[str, np.ndarray]],
        fps: float,
    ) -> None:
        """Open one sink per camera, sized from the frames currently available."""
        self._frames = frames
        self._period = 1.0 / fps
        self._stop = threading.Event()
        self._error: Optional[BaseException] = None
        out_dir = Path(out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        first = frames()
        self.paths = {name: out_dir / f"{name}.mp4" for name in first}
        self._sinks = {
            name: VideoSink(self.paths[name], f.shape[1], f.shape[0], fps) for name, f in first.items()
        }
        self.frame_count = 0
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        """Begin writing frames."""
        self._thread.start()

    def _run(self) -> None:
        """Write one frame per camera each tick until stopped."""
        next_tick = time.monotonic()
        try:
            while not self._stop.is_set():
                for name, frame in self._frames().items():
                    self._sinks[name].write(frame)
                self.frame_count += 1
                next_tick += self._period
                delay = next_tick - time.monotonic()
                if delay > 0:
                    self._stop.wait(delay)
                else:
                    next_tick = time.monotonic()
        except BaseException as exc:  # noqa: BLE001
            self._error = exc

    def stop(self) -> Dict[str, Path]:
        """Stop, close every sink and return the video path per camera."""
        self._stop.set()
        self._thread.join()
        for sink in self._sinks.values():
            sink.close()
        if self._error is not None:
            raise RuntimeError(f"video writer failed: {self._error}") from self._error
        return self.paths


@dataclass
class RecordResult:
    """What one recorded episode looked like."""

    steps: int
    elapsed_s: float
    rate_hz: float
    videos: Dict[str, Path]


class EpisodeRecorder:
    """Records episodes from a source (anything with `snapshot()`) into an EpisodeWriter."""

    def __init__(
        self,
        profile,
        source,
        *,
        teleop_actions: bool = False,
        source_timeout: float = 0.5,
        command_timeout: float = 0.5,
        camera_fps: float = 30.0,
        tmp_dir: Path = Path("/tmp/datahive_video"),
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        """Pick the extractors the profile asks for."""
        check_profile_supported(profile, teleop_actions)
        self.profile = profile
        self.source = source
        self.source_timeout = source_timeout
        self.command_timeout = command_timeout if teleop_actions else 0.0
        self.camera_fps = camera_fps
        self.tmp_dir = Path(tmp_dir)
        self.clock = clock
        self.action_extractors = dict(ACTION_EXTRACTORS)
        if teleop_actions:
            self.action_extractors.update(TELEOP_ACTION_EXTRACTORS)

    def _check_fresh(self, snap: Snapshot) -> None:
        """Raise if any source has gone quiet for longer than its limit."""
        now = time.time()
        if self.source_timeout:
            stale = [n for n, t in snap.times.items() if now - t > self.source_timeout]
            if stale:
                raise StaleSource(
                    f"no input from {', '.join(stale)} for over {self.source_timeout:.2f}s; recording on "
                    "would repeat the last frame/pose, which still passes validation"
                )
        if self.command_timeout and snap.command.age(now) > self.command_timeout:
            raise StaleCommand(
                f"no teleop command for {snap.command.age(now):.2f}s (limit {self.command_timeout:.2f}s)"
            )

    def _action(self, snap: Snapshot) -> np.ndarray:
        """Concatenate the profile's action_space entries into one target vector."""
        parts = [
            np.ravel(self.action_extractors[k](snap.state, snap.gripper, snap.command))
            for k in self.profile.action_space
        ]
        return np.concatenate(parts).astype(np.float64)

    def record(
        self,
        writer,
        *,
        stop: Callable[[], bool] = lambda: False,
        max_seconds: float = MAX_EPISODE_S,
        control_mode: Optional[str] = None,
    ) -> RecordResult:
        """Record until `stop()` or `max_seconds`; attach the videos to `writer` on success."""
        max_seconds = min(max_seconds, MAX_EPISODE_S)
        control_mode = control_mode or self.profile.control_mode or "cartesian_impedance"
        period = 1.0 / float(self.profile.control_freq)
        video_dir = self.tmp_dir / writer.episode_id
        video = VideoRecorder(video_dir, lambda: self.source.snapshot().frames, self.camera_fps)
        first_action: Optional[np.ndarray] = None
        varied = False
        steps = 0
        started = self.clock()
        next_tick = started
        video.start()
        try:
            while True:
                snap = self.source.snapshot()
                self._check_fresh(snap)
                t = self.clock() - started
                action = self._action(snap)
                writer.append_proprioception(
                    timestamp=t,
                    provenance=PROVENANCE,
                    **{k: STATE_EXTRACTORS[k](snap.state, snap.gripper) for k in STATE_EXTRACTORS},
                )
                writer.append_command(timestamp=t, target=action, control_mode=control_mode)
                steps += 1
                if self.command_timeout:
                    if first_action is None:
                        first_action = action
                    elif not varied and not np.allclose(action, first_action, atol=1e-6):
                        varied = True
                if t >= max_seconds or stop():
                    break
                next_tick += period
                delay = next_tick - self.clock()
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_tick = self.clock()
        except BaseException:
            video.stop()
            shutil.rmtree(video_dir, ignore_errors=True)
            raise
        elapsed = self.clock() - started
        videos = video.stop()
        if elapsed < MIN_EPISODE_S:
            shutil.rmtree(video_dir, ignore_errors=True)
            raise TooShort(f"episode lasted {elapsed:.2f}s; the minimum is {MIN_EPISODE_S:.0f}s")
        if self.command_timeout and steps > 1 and not varied:
            shutil.rmtree(video_dir, ignore_errors=True)
            raise ConstantAction(f"the action never changed across {steps} steps")
        for name, path in videos.items():
            writer.attach_video(name, path, move=True)
        shutil.rmtree(video_dir, ignore_errors=True)
        return RecordResult(steps, elapsed, steps / elapsed if elapsed else 0.0, videos)


def discard_episode(writer) -> None:
    """Close and delete an episode's .h5 and videos."""
    writer.close()
    writer.paths.h5.unlink(missing_ok=True)
    for path in list(writer.paths.videos.values()):
        Path(path).unlink(missing_ok=True)
