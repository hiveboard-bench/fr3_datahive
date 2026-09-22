import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from datahive.episode import EpisodeWriter
from datahive.profile import incompleteness_problems, load_profile
from datahive.check import check_episode
from fr3_datahive.convert import image_to_rgb, pose_from_column_major
from fr3_datahive.recorder import (
    Command,
    ConstantAction,
    EpisodeRecorder,
    Snapshot,
    StaleSource,
    TooShort,
    discard_episode,
)

PROFILE = Path(__file__).resolve().parents[1] / "ros/fr3_datahive/config/robot_profile.yaml"


def fake_state(shift=0.0):
    t = np.eye(4)
    t[:3, 3] = [0.3 + shift, 0.0, 0.5]
    return SimpleNamespace(
        q=np.zeros(7), dq=np.zeros(7), tau_J=np.zeros(7), q_d=np.zeros(7), dq_d=np.zeros(7),
        O_T_EE=t.flatten(order="F"), O_T_EE_d=t.flatten(order="F"),
    )


class FakeSource:
    def __init__(self, moving=True, stale_after=None):
        self.moving, self.stale_after, self.t0 = moving, stale_after, time.time()
        self.frame = np.random.randint(0, 255, (240, 320, 3), dtype=np.uint8)

    def snapshot(self):
        now = time.time()
        shift = 0.1 * (now - self.t0) if self.moving else 0.0
        times = {"franka_state": now, "gripper": now, "camera:front": now}
        if self.stale_after is not None and now - self.t0 > self.stale_after:
            times["camera:front"] = now - 5
        return Snapshot(fake_state(shift), 0.04, {"front": self.frame}, Command(), times)


@pytest.fixture
def samples(tmp_path):
    raw = yaml.safe_load(PROFILE.read_text())
    raw["cameras"] = [{"name": "front", "position": "a", "orientation": "b"}]
    (tmp_path / "robot_profile.yaml").write_text(yaml.safe_dump(raw))
    return tmp_path


def writer_for(samples, name="ep"):
    return EpisodeWriter(samples, "s1", name, trial_id="1", lab_id="lab",
                         profile=load_profile(samples), collection_mode="automatic")


def recorder(samples, source, **kw):
    return EpisodeRecorder(load_profile(samples), source, tmp_dir=samples / "tmp", **kw)


def test_profile_template_is_complete():
    assert incompleteness_problems(yaml.safe_load(PROFILE.read_text())) == []


def test_pose_identity_and_rotation():
    assert np.allclose(pose_from_column_major(np.eye(4).flatten(order="F")), [0, 0, 0, 0, 0, 0, 1])
    rz = np.eye(4)
    rz[:2, :2] = [[0, -1], [1, 0]]
    rz[:3, 3] = [1, 2, 3]
    pose = pose_from_column_major(rz.flatten(order="F"))
    assert np.allclose(pose, [1, 2, 3, 0, 0, np.sqrt(0.5), np.sqrt(0.5)])


def test_image_decode_handles_bgr_and_row_padding():
    rgb = np.arange(2 * 2 * 3, dtype=np.uint8).reshape(2, 2, 3)
    padded = np.zeros((2, 8), dtype=np.uint8)
    padded[:, :6] = rgb[:, :, ::-1].reshape(2, 6)
    assert np.array_equal(image_to_rgb(2, 2, "bgr8", 8, padded.tobytes()), rgb)
    with pytest.raises(ValueError):
        image_to_rgb(2, 2, "16UC1", 8, bytes(16))


def test_records_a_valid_episode(samples):
    w = writer_for(samples)
    rec = recorder(samples, FakeSource(), teleop_actions=False)
    result = rec.record(w, max_seconds=2.2)
    w.close()
    assert result.rate_hz >= 100
    assert w.paths.videos["front"].exists()
    report = check_episode(samples, "ep")
    assert report["ok"], report["problems"]
    assert report["stats"]["sample_rate_hz"] >= 100


def test_too_short_is_rejected(samples):
    w = writer_for(samples)
    with pytest.raises(TooShort):
        recorder(samples, FakeSource()).record(w, max_seconds=0.3)
    discard_episode(w)
    assert not w.paths.h5.exists()


def test_stale_camera_discards(samples):
    w = writer_for(samples)
    with pytest.raises(StaleSource):
        recorder(samples, FakeSource(stale_after=0.5)).record(w, max_seconds=3)
    discard_episode(w)


def test_constant_teleop_action_discards(samples):
    src = FakeSource(moving=False)
    src.snapshot = lambda: Snapshot(
        fake_state(), 0.04, {"front": src.frame},
        Command(np.array([0.3, 0, 0.5, 0, 0, 0, 1.0]), 0.04, time.time(), time.time()),
        {"franka_state": time.time(), "gripper": time.time(), "camera:front": time.time()},
    )
    w = writer_for(samples)
    with pytest.raises(ConstantAction):
        recorder(samples, src, teleop_actions=True).record(w, max_seconds=1.5)
    discard_episode(w)


def test_camera_topics_match_ignoring_case():
    from fr3_datahive.recorder import resolve_camera_topics

    got = resolve_camera_topics(["Front", "Wrist"], {"front": "/a", "wrist": "/b", "top": "/c"})
    assert got == {"Front": "/a", "Wrist": "/b"}
    with pytest.raises(SystemExit):
        resolve_camera_topics(["External"], {"front": "/a"})
