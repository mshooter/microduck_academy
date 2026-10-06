"""The fetch_ball HAL drives the server the way student code would, over a real socket.

HAL.py connects to the default socket when imported, like Academy's HALs. Here
it is pointed at a test server on a temporary socket instead, by swapping in a
DuckClient bound to that path before the import. One server and one HAL for
the whole module; physics tests run in real time, so this file takes a few seconds.
"""
import contextlib
import functools
import importlib.util
import io
import os
import shutil
import tempfile
import time
from pathlib import Path

import numpy as np
import pytest

from microduck_academy import duckd_sim
from microduck_academy.hal_interfaces import client
from microduck_academy.hal_interfaces.odometry import Pose3d

HAL_PATH = Path(__file__).resolve().parents[1] / "exercises" / "fetch_ball" / "python_template" / "HAL.py"


@pytest.fixture(scope="module")
def server():
    # Short path on purpose: macOS limits Unix socket paths to ~104 bytes. See test_contract.py.
    folder = tempfile.mkdtemp(prefix="duck-", dir="/tmp")
    path = os.path.join(folder, "duck.sock")
    try:
        with contextlib.redirect_stdout(io.StringIO()):   # Pollen's loop prints on every command
            with duckd_sim.DuckServer(path) as s:
                yield s
    finally:
        shutil.rmtree(folder, ignore_errors=True)


@pytest.fixture(scope="module")
def hal(server):
    """Import HAL.py as student code would, connected to the test server."""
    with pytest.MonkeyPatch.context() as mp:
        mp.setattr(client, "DuckClient", functools.partial(client.DuckClient, server.path))
        spec = importlib.util.spec_from_file_location("HAL", HAL_PATH)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module) # connects, says hello, subscribes
    yield module
    module.motors.close() # stop resending before closing the socket
    module.duck.close()


@pytest.fixture
def fresh(hal, server):
    """Each test starts standing at the origin with no velocity command."""
    hal.stop()
    t_before = wait_sim_seconds(hal, 0.2)["t"]           # > 0, even on a server that has just started
    server.sim.reset()                                    # also rewinds the server's clock to 0
    deadline = time.monotonic() + 2.0
    while hal.duck.state()["t"] >= t_before:              # skip frames sent before the reset
        assert time.monotonic() < deadline, "no robot.state arrived after sim.reset"
        time.sleep(0.01)
    wait_sim_seconds(hal, 0.3)
    return hal


def wait_sim_seconds(hal, seconds, wall_limit=10.0):
    """Wait on the server's clock, not the wall's: sim time falls behind on a slow CI runner."""
    deadline = time.monotonic() + wall_limit
    while hal.duck.state() is None:
        assert time.monotonic() < deadline, "no robot.state arrived: HAL.py did not subscribe"
        time.sleep(0.01)
    t0 = hal.duck.state()["t"]
    while hal.duck.state()["t"] - t0 < seconds:
        assert time.monotonic() < deadline, f"only {hal.duck.state()['t'] - t0:.2f} s simulated in {wall_limit} s"
        time.sleep(0.02)
    return hal.duck.state()


# -- import and sensors ------------------------------------------------------------

def test_import_connects_and_state_streams(fresh):
    pose = fresh.getPose3d()
    assert isinstance(pose, Pose3d)
    assert abs(pose.x) < 0.05 and abs(pose.y) < 0.05, "reset puts the duck at the origin"
    assert pose.timeStamp > 0, "timeStamp comes from robot.state's t"
    assert fresh.isFallen() is False
    assert fresh.getPolicy() in {"walk", "stand", "sit"}


def test_get_image_is_bgr_640x360(fresh):
    try:
        image = fresh.getImage()
    except RuntimeError as e:
        pytest.skip(f"no OpenGL context here: {e}")
    assert image.shape == (360, 640, 3)
    assert image.dtype == np.uint8
    assert image.flags["C_CONTIGUOUS"], "the BGR flip must be copied, OpenCV rejects negative strides"


# -- motors, real time --------------------------------------------------------------

def test_set_v_once_keeps_walking_past_the_expiry(fresh):
    """One setV call, no resend from the test: MotorsDuck's thread must keep robot.move alive."""
    x0 = fresh.getPose3d().x
    fresh.setV(0.3)
    state = wait_sim_seconds(fresh, 3.0)                  # 6 x MOVE_EXPIRY
    assert state["move"]["applied"][0] == pytest.approx(0.3)
    assert "limited_by" not in state["move"], f"move was limited: {state['move'].get('limited_by')}"
    assert fresh.getPose3d().x - x0 > 0.2, "0.3 m/s for 3 s (measured ~0.12 m/s achieved)"
    assert fresh.isFallen() is False


def test_set_w_is_sent_as_vyaw(fresh):
    fresh.setW(0.5)
    state = wait_sim_seconds(fresh, 1.0)
    assert state["move"]["requested"][2] == pytest.approx(0.5)


def test_stop_zeroes_the_velocity(fresh):
    fresh.setV(0.3)
    wait_sim_seconds(fresh, 0.5)
    fresh.stop()
    state = wait_sim_seconds(fresh, 1.0)                  # longer than one resend period: the loop sends zeros now
    assert state["move"]["requested"] == [0.0, 0.0, 0.0]
    assert state["move"]["applied"] == [0.0, 0.0, 0.0]


# -- head and skills ----------------------------------------------------------------

def test_look_at_returns_a_bool(fresh):
    assert fresh.lookAt(0.3, 0.0, -0.12) is True, "a point just ahead on the floor is within reach"


def test_do_skill_accepts_a_known_skill_and_rejects_a_typo(fresh):
    assert fresh.doSkill("kick_left") is True
    with pytest.raises(RuntimeError, match="-32602"):
        fresh.doSkill("kick_lef")
