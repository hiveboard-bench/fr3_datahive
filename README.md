# fr3_datahive

Collect [DataHive](https://github.com/hiveboard-bench/DataHive) episodes with a **Franka Research 3**,
in a ready-to-run Docker container (ROS 1 Noetic + libfranka + franka_ros + `datahive-tools`).
It records the arm state,
the gripper, the command and one video per camera at 200 Hz, and writes them in the format
`datahive check / annotate / validate / upload` expects.

Two ways to collect, matching the two modes of the Runner:

| | Who runs the trial | What the robot side does | Then |
|---|---|---|---|
| **Manual** (`make manual`) | you, with the browser timer | records an episode to a folder on Enter | upload the `.h5` + one video per camera in the Runner (Manual) |
| **Automatic** (`make auto`) | you press **Send to robot** in the Runner | claims the trial, records it, hands it back | annotate it in Annotate |

Started from the collection, control and teleop code of a lab's Oopsie Data FR3 rig, rebuilt on
DataHive's `EpisodeWriter` and `CollectClient`.

## Requirements

- Docker, a Franka Research 3 with FCI enabled, and a real-time kernel on the host (needed by
  `franka_control`; not by anything else here).
- Up to three UVC cameras (front, top, wrist). Each camera's name in
  `ros/fr3_datahive/config/robot_profile.yaml` must match a `--camera` topic in
  `launch/record.launch`.
- A Hugging Face token for uploading (`make init`).

## First time

```bash
cd fr3_datahive
make build            # ~30-60 min the first time: compiles libfranka and franka_ros
make init             # lab id + Hugging Face token, stored in ./datahive-config
make profile          # copies the FR3 profile to ./samples/robot_profile.yaml
$EDITOR samples/robot_profile.yaml   # review every line marked EDIT
make cameras-list     # find the /dev/v4l paths of your cameras
```

`samples/`, `recordings/` and `datahive-config/` are bind-mounted from this folder, so your data
lives on the host.

## Record: one command

```bash
make collect MODE=manual ARGS="robot_ip:=172.16.0.2 front_device:=/dev/v4l/by-id/... top_device:=... wrist_device:=..."
make collect MODE=auto   ARGS="..."
```

This starts everything: `franka_control` + gripper, the collision-reflex thresholds, the three
cameras, the Cartesian impedance controller, the joystick teleop, the DataHive GUI
(`http://localhost:8000`) and the recorder. The action recorded is what the teleop commanded.

The arm does not move until you press **Start** on the pad (**Back** disengages). Pad layout and
speeds are in `ros/fr3_teleop/config/teleop.yaml` (default: an 8BitDo pad on `/dev/input/js0`;
left stick = XY, LT/LB = down/up, right stick = tilt, RT/RB = yaw, A/B = close/open gripper,
X held 2 s = go home, button 2 = end the current episode).

Useful variants: `teleop:=none` (controller only, you drive from another source),
`cameras:=false`, and `teleop_actions:=false` (record FrankaState's desired pose instead).
The pieces are also available separately: `make bringup`, `make teleop`, `make engage`,
`make interface`, `make manual`, `make auto`, `make home`.

### Manual mode

```bash
make manual                                   # (or use `make collect MODE=manual`)  ARGS="extra_args:='--count 10 --max-seconds 120'"
```

1. In the Runner create a session and pick **Manual** for the trial.
2. Start the browser timer and run the trial.
3. In the terminal press **Enter** to start recording and **Enter** to stop.
4. Upload the printed `recordings/manual/episodes/*.h5` and the three `*_cam_*.mp4` files in the
   Runner, then record the outcome.

The recorder writes `collection_mode: manual`; the Runner rebuilds the header from the session and
your profile when you upload, so only the data in the file matters.

### Automatic mode

```bash
make auto
```

Press **Send to robot** in the Runner. The recorder claims the trial, records until the task's
timeout (or **Enter**, `rostopic pub /datahive/stop std_msgs/Bool "data: true" -1`, or
`pkill -USR1 -f record.py`), and hands the episode to Annotate. Annotate it there; the next trial is released
when you save. A trial that fails a safety check (below) is deleted and the browser trial is
aborted, so just send it again.

### After recording

```bash
make check                          # frames, rate, gaps, videos for everything in samples/
make check SAMPLES=/data/recordings # manual recordings
```

Then annotate, `datahive validate` and `datahive upload` as usual (from the GUI, or `make shell`).

## What is recorded

| Field | Source |
|---|---|
| `observations/robot_states/joint_position` (7) | `FrankaState.q` |
| `observations/robot_states/joint_velocity` (7) | `FrankaState.dq` (marked `estimated`, the controller filters it) |
| `observations/robot_states/joint_torque_or_current` (7) | `FrankaState.tau_J` |
| `observations/robot_states/ee_pose` (7) | `O_T_EE` as `x y z qx qy qz qw` (marked `estimated`, computed from the joints) |
| `observations/robot_states/ee_state` (1) | total finger opening (m) from `/franka_gripper/joint_states` |
| `actions/cartesian_position` (7) | commanded Cartesian pose, from teleop when enabled or `O_T_EE_d` otherwise |
| `actions/gripper_position` (1) | commanded gripper width (m) |
| `observations/video_paths/<camera>` | relative filename of that camera's recorded MP4 |
| videos | one H.264 mp4 per camera, 30 fps, written with ffmpeg |

Provenance labels live in `src/fr3_datahive/convert.py`. Change `action_space` in the profile to
record joint targets instead (`joint_position`, `joint_velocity`), and `action_joint_names` with it.

**What "action" means.** By default the action is FrankaState's *desired* pose (`O_T_EE_d`). Under
an impedance controller with no motion generator (hand-guiding included), that is not an
independent signal. If your teleop publishes the commanded pose and gripper width, pass
`teleop_actions:=true` (topics: `--teleop-pose-topic`, default
`/fr3_cartesian_impedance_controller/equilibrium_pose`, and `--teleop-gripper-topic`, default
`/teleop/gripper_width_command`). The recorder then stores exactly that.

## Safety checks

An episode is **discarded**, not saved, when:

- a camera, the gripper or `franka_control` stops publishing for more than 0.5 s (otherwise the
  last frame/pose would be repeated and still pass validation),
- with `teleop_actions`, the command stream goes stale, or never changes,
- it lasts under 1 s.

## Layout

```
fr3_datahive/
  Dockerfile, Makefile, docker/           image and entrypoint
  ros/fr3_datahive/                       bring-up, cameras, recorder (mounted into the container)
    scripts/record.py                     `auto` and `manual` entry point
    scripts/uvc_camera.py                 UVC camera -> sensor_msgs/Image
    launch/{bringup,cameras,record,collect}.launch
    config/robot_profile.yaml             FR3 profile template
    src/fr3_datahive/                     recorder (ROS-free), ROS input buffers, conversions
  ros/fr3_control/                        Cartesian impedance controller (C++)
  ros/fr3_teleop/                         joystick input + follower for the controller
  tests/                                  no ROS needed
```

The recorder runs under a Python 3.10 venv (`/opt/datahive-venv`) because `datahive-tools` needs
3.10 and Noetic ships 3.8. It requires `datahive-tools` 0.1.7 or newer for the current HDF5
layout. It borrows the pure-Python ROS packages (`rospy`, message types) and
decodes images itself, so `cv_bridge` is not used.

## Customising

- **Different cameras / count**: edit `cameras` in the profile and the `--camera` lines in
  `launch/record.launch`; `cameras.launch` has one node per camera.
- **Stereo camera (e.g. ZED)**: `cameras.launch ... stereo:=true` publishes the left half.
- **Rate**: `control_freq` in the profile. Must stay ≥ 100; check with `make check` after the
  first episode and read `sample_rate_hz` and `timestamp_gaps`.
- **Your own controller**: leave bring-up out and start yours; the recorder only needs
  `/franka_state_controller/franka_states` and `/franka_gripper/joint_states`
  (override with `--franka-state-topic` / `--gripper-topic` through `ARGS="extra_args:=..."`).

## Development

```bash
pip install "datahive-tools[video]>=0.1.7" pytest
pytest tests
```

The tests cover conversions and the recorder with a fake source (an episode it records passes
`datahive check`). They do not need ROS or a robot.

## Status

This has recorded real episodes end to end (manual and automatic, teleop-commanded actions,
three cameras) on one FR3 rig, but has not been used across labs yet. If something in your setup
doesn't fit -- a different gripper, camera count, controller -- open an issue; the goal is for
this to work for any FR3 + DataHive lab, not just the one it started on.

## License

MIT, see [LICENSE](LICENSE). `fr3_control`'s Cartesian impedance controller is forked from
[serl_franka_controllers](https://github.com/rail-berkeley/serl_franka_controllers) (also MIT).
