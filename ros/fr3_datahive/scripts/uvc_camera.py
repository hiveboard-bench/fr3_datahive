#!/usr/bin/env python3
"""Publish frames from a UVC camera as sensor_msgs/Image."""

from __future__ import annotations

import argparse
import sys

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image


def ros_argv(argv):
    """Drop the __name:=/__log:= arguments roslaunch appends to node args."""
    return [a for a in argv if ":=" not in a]


def output_frame(frame: np.ndarray, stereo_side_by_side: bool) -> np.ndarray:
    """Return a full UVC frame, or the left half of a stereo frame."""
    if stereo_side_by_side:
        return frame[:, : frame.shape[1] // 2]
    return frame


def resize_frame(frame: np.ndarray, width: int, height: int) -> np.ndarray:
    """Center-crop and resize a frame without changing its aspect ratio."""
    source_height, source_width = frame.shape[:2]
    target_ratio = width / float(height)
    source_ratio = source_width / float(source_height)
    if source_ratio > target_ratio:
        crop_width = int(round(source_height * target_ratio))
        left = (source_width - crop_width) // 2
        frame = frame[:, left : left + crop_width]
    elif source_ratio < target_ratio:
        crop_height = int(round(source_width / target_ratio))
        top = (source_height - crop_height) // 2
        frame = frame[top : top + crop_height, :]
    return cv2.resize(frame, (width, height))


def main(argv=None) -> int:
    """Open the UVC device and publish frames until shutdown."""
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--device", required=True, help="/dev/v4l/by-id/... path (preferred) or /dev/videoN"
    )
    parser.add_argument("--topic", required=True, help="topic to publish sensor_msgs/Image on")
    parser.add_argument("--frame-id", default="camera")
    parser.add_argument(
        "--stereo-side-by-side",
        action="store_true",
        help="publish the left half of a side-by-side stereo frame",
    )
    parser.add_argument("--width", type=int, default=2560, help="requested UVC capture width")
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=float, default=30.0, help="capture rate")
    parser.add_argument(
        "--publish-rate",
        type=float,
        default=0.0,
        help="publish rate; 0 means publish every captured frame",
    )
    parser.add_argument(
        "--output-width",
        type=int,
        default=640,
        help="resize the selected image to this width before publishing (0 = native)",
    )
    parser.add_argument(
        "--output-height",
        type=int,
        default=0,
        help="resize the selected image to this height (0 = preserve aspect ratio)",
    )
    parser.add_argument("--open-timeout", type=float, default=10.0)
    args = parser.parse_args(ros_argv(sys.argv[1:] if argv is None else argv))

    rospy.init_node("uvc_camera", anonymous=True)

    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not cap.isOpened():
        rospy.logfatal(
            "could not open %s. Check `ls /dev/v4l/by-id/` and that the "
            "container was started with access to /dev/video*",
            args.device,
        )
        return 1
    rospy.on_shutdown(cap.release)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if (actual_w, actual_h) != (args.width, args.height):
        rospy.logwarn(
            "requested %dx%d but the device gave %dx%d", args.width, args.height, actual_w, actual_h
        )
    output_w = actual_w // 2 if args.stereo_side_by_side else actual_w
    mode = "left stereo image" if args.stereo_side_by_side else "full frame"
    rospy.loginfo(
        "%s -> %s at %dx%d (%s: %dx%d)",
        args.device,
        args.topic,
        actual_w,
        actual_h,
        mode,
        output_w,
        actual_h,
    )

    pub = rospy.Publisher(args.topic, Image, queue_size=1)
    rate = rospy.Rate(args.publish_rate) if args.publish_rate > 0 else None

    failures = 0
    while not rospy.is_shutdown():
        ok, frame = cap.read()
        captured = rospy.Time.now()
        if not ok:
            failures += 1
            if failures % 30 == 1:
                rospy.logwarn("frame grab failed on %s (%d times)", args.device, failures)
            if failures > 300:
                rospy.logfatal("giving up on %s after %d failed grabs", args.device, failures)
                cap.release()
                return 1
            continue
        failures = 0

        img = output_frame(frame, args.stereo_side_by_side)
        if args.output_width and args.output_height:
            if img.shape[:2] != (args.output_height, args.output_width):
                img = resize_frame(img, args.output_width, args.output_height)
        elif args.output_width and img.shape[1] != args.output_width:
            scale = args.output_width / float(img.shape[1])
            img = cv2.resize(img, (args.output_width, int(round(img.shape[0] * scale))))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        msg = Image()
        msg.header.stamp = captured
        msg.header.frame_id = args.frame_id
        msg.height, msg.width = img.shape[:2]
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = np.ascontiguousarray(img).tobytes()
        pub.publish(msg)

        if rate is not None:
            rate.sleep()

    cap.release()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        pass
