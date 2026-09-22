#!/usr/bin/env python3
"""Record FR3 episodes for DataHive: `auto` follows the browser's Runner, `manual` records files to upload."""

from __future__ import annotations

import argparse
import select
import signal
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import List, Optional

import rospy
from datahive.collect import CollectClient, CollectError
from datahive.episode import EpisodeWriter
from datahive.profile import load_profile
from std_msgs.msg import Bool

from fr3_datahive.recorder import (
    EpisodeDiscarded,
    EpisodeRecorder,
    discard_episode,
    resolve_camera_topics,
)
from fr3_datahive.ros_source import RosSource

STOP = threading.Event()


def ros_argv(argv: List[str]) -> List[str]:
    """Drop the __name:=/__log:= arguments roslaunch appends to node args."""
    return [a for a in argv if ":=" not in a]


def parse_cameras(pairs: List[str]) -> dict:
    """Parse NAME:TOPIC pairs into a mapping."""
    out = {}
    for pair in pairs:
        if ":" not in pair:
            raise SystemExit(f"--camera expects NAME:TOPIC, got {pair!r}")
        name, topic = pair.split(":", 1)
        out[name.strip()] = topic.strip()
    return out


def enter_pressed() -> bool:
    """True once Enter has been typed on an interactive terminal (consumes the line)."""
    if sys.stdin.isatty() and select.select([sys.stdin], [], [], 0)[0]:
        sys.stdin.readline()
        return True
    return False


def drain_stdin() -> None:
    """Discard anything typed before an episode starts."""
    while sys.stdin.isatty() and select.select([sys.stdin], [], [], 0)[0]:
        sys.stdin.readline()


def stop_requested() -> bool:
    """True when Enter, SIGUSR1 or the stop topic asked to end the episode."""
    return STOP.is_set() or enter_pressed()


def add_common(parser: argparse.ArgumentParser) -> None:
    """Options shared by both modes."""
    parser.add_argument("--samples-dir", default="/data/samples", help="DataHive samples/ tree")
    parser.add_argument("--camera", action="append", default=[], metavar="NAME:TOPIC")
    parser.add_argument("--franka-state-topic", default="/franka_state_controller/franka_states")
    parser.add_argument("--gripper-topic", default="/franka_gripper/joint_states")
    parser.add_argument("--teleop-actions", action="store_true",
                        help="record what the teleop node commanded instead of FrankaState's desired pose")
    parser.add_argument("--teleop-pose-topic", default="/fr3_cartesian_impedance_controller/equilibrium_pose")
    parser.add_argument("--teleop-gripper-topic", default="/teleop/gripper_width_command")
    parser.add_argument("--stop-topic", action="append", default=None,
                        help="std_msgs/Bool topic whose True ends the episode (repeatable; default /datahive/stop and the joystick /teleop/record_trigger)")
    parser.add_argument("--source-timeout", type=float, default=0.5)
    parser.add_argument("--command-timeout", type=float, default=0.5)
    parser.add_argument("--camera-fps", type=float, default=30.0)
    parser.add_argument("--wait-timeout", type=float, default=30.0)


def build(args) -> EpisodeRecorder:
    """Load the profile, subscribe to the topics and wait until every input delivers."""
    profile = load_profile(Path(args.samples_dir))
    names = [c["name"] for c in profile.cameras]
    rospy.init_node("fr3_datahive_record", anonymous=True, disable_signals=True)
    for topic in args.stop_topic or ["/datahive/stop", "/teleop/record_trigger"]:
        rospy.Subscriber(topic, Bool, lambda m: STOP.set() if m.data else None, queue_size=1)
    teleop = (
        {"pose": args.teleop_pose_topic, "gripper": args.teleop_gripper_topic}
        if args.teleop_actions else None
    )
    source = RosSource(names, resolve_camera_topics(names, parse_cameras(args.camera)),
                       args.franka_state_topic,
                       args.gripper_topic, teleop)
    recorder = EpisodeRecorder(
        profile, source, teleop_actions=args.teleop_actions, source_timeout=args.source_timeout,
        command_timeout=args.command_timeout, camera_fps=args.camera_fps,
    )
    print("waiting for robot state, gripper and cameras ...")
    source.wait_ready(args.wait_timeout)
    print(f"ready: {profile.control_freq:g} Hz, cameras {names}")
    return recorder


def run_auto(args) -> int:
    """Serve trials sent from the browser's Runner until interrupted."""
    recorder = build(args)
    client = CollectClient(Path(args.samples_dir), base_url=args.base_url)
    print(f"automatic mode: press 'Send to robot' in the Runner ({args.base_url}); Ctrl-C to quit")
    try:
        while not rospy.is_shutdown():
            try:
                task = client.wait_for_task(timeout_s=5.0)
            except CollectError:
                continue
            except OSError as exc:
                print(f"cannot reach {args.base_url}: {exc}; is `datahive interface` running?")
                time.sleep(2.0)
                continue
            writer = client.new_writer(task, profile=recorder.profile)
            STOP.clear()
            drain_stdin()
            print(f"\ntrial {task['trial_id']} [{task['name']}] -> {writer.episode_id}"
                  f" (timeout {task['timeout']}s, Enter stops)")
            try:
                result = recorder.record(writer, stop=stop_requested, max_seconds=float(task["timeout"]))
            except EpisodeDiscarded as exc:
                discard_episode(writer)
                client._request("POST", "/api/collect/abort", {})
                print(f"  DISCARDED: {exc}\n  the browser trial was aborted; send it again")
                continue
            except BaseException:
                discard_episode(writer)
                client._request("POST", "/api/collect/abort", {})
                raise
            print(f"  {result.steps} steps in {result.elapsed_s:.1f}s ({result.rate_hz:.0f} Hz); "
                  "annotate it in the browser")
            if not client.finish(writer):
                print("  the operator aborted this trial")
    except KeyboardInterrupt:
        print("\nstopped")
    return 0


def run_manual(args) -> int:
    """Record standalone episodes to upload through the Runner's Manual mode."""
    recorder = build(args)
    out = Path(args.out_dir)
    print(f"manual mode: files go to {out}; upload each .h5 and its videos in the Runner (Manual)")
    n = args.start_index
    try:
        while not rospy.is_shutdown() and (args.count == 0 or n < args.start_index + args.count):
            input(f"\n[{n}] Enter to start recording, Ctrl-C to quit: ")
            stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")
            writer = EpisodeWriter(out, "manual", f"t{n}_{stamp}", trial_id=str(n), lab_id="manual",
                                   profile=recorder.profile, collection_mode="manual")
            STOP.clear()
            print(f"  recording (Enter to stop, max {args.max_seconds:.0f}s) ...")
            try:
                result = recorder.record(writer, stop=stop_requested, max_seconds=args.max_seconds)
            except EpisodeDiscarded as exc:
                discard_episode(writer)
                print(f"  DISCARDED: {exc}")
                continue
            writer.close()
            print(f"  {result.steps} steps in {result.elapsed_s:.1f}s ({result.rate_hz:.0f} Hz)")
            print(f"  h5:     {writer.paths.h5}")
            for cam, path in writer.paths.videos.items():
                print(f"  video:  {cam}: {path}")
            n += 1
    except (KeyboardInterrupt, EOFError):
        print("\nstopped")
    return 0


def main(argv: Optional[List[str]] = None) -> int:
    """Parse the mode and run it."""
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="mode", required=True)
    auto = sub.add_parser("auto", help="record the trials the browser sends")
    add_common(auto)
    auto.add_argument("--base-url", default="http://127.0.0.1:8000", help="datahive interface URL")
    manual = sub.add_parser("manual", help="record episodes to files for manual upload")
    add_common(manual)
    manual.add_argument("--out-dir", default="/data/recordings")
    manual.add_argument("--max-seconds", type=float, default=180.0)
    manual.add_argument("--count", type=int, default=0, help="stop after N episodes; 0 = until Ctrl-C")
    manual.add_argument("--start-index", type=int, default=1)
    args = parser.parse_args(ros_argv(sys.argv[1:] if argv is None else argv))
    signal.signal(signal.SIGUSR1, lambda *_: STOP.set())
    return run_auto(args) if args.mode == "auto" else run_manual(args)


if __name__ == "__main__":
    sys.exit(main())
