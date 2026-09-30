"""The server and the example client agree with docs/api.md, over a real socket.

One server is started for the whole module (loading the model and policies takes
about a second). Physics tests run in real time, so this file takes a few seconds.
"""
import base64
import contextlib
import io
import os
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import pytest

from examples.client import DuckClient
from microduck_academy import duckd_sim

ROOT = Path(__file__).resolve().parents[1]

# docs/api.md, section 2, robot.state: the keys every frame carries.
STATE_KEYS = {"t", "move", "head", "policy", "safety", "loop", "joints", "targets", "odom"}
SAFETY_KEYS = {"fallen", "limp", "gravity"}


@pytest.fixture(scope="module")
def server():
    # A short path on purpose: Unix socket paths are limited to ~104 bytes on macOS,
    # and pytest's tmp_path there is longer than that.
    folder = tempfile.mkdtemp(prefix="duck-", dir="/tmp")
    path = os.path.join(folder, "duck.sock")
    try:
        with contextlib.redirect_stdout(io.StringIO()):   # Pollen's loop prints on every command
            with duckd_sim.DuckServer(path) as s:
                yield s
    finally:
        shutil.rmtree(folder, ignore_errors=True)


@pytest.fixture
def duck(server):
    server.sim.reset()
    d = DuckClient(server.path)
    d.call("hello", {"api_version": 16})
    yield d
    d.close()                                             # so the server stops streaming to a dead socket


def wait_for_state(duck, timeout=2.0):
    deadline = time.monotonic() + timeout
    while duck.state() is None and time.monotonic() < deadline:
        time.sleep(0.01)
    assert duck.state() is not None, "no robot.state notification arrived"
    return duck.state()


# -- wire shapes ------------------------------------------------------------------

def test_hello_reply_has_documented_fields(duck):
    reply = duck.call("hello", {"api_version": 16})
    assert set(reply) == {"api_version", "daemon_version", "revision"}
    assert reply["api_version"] == 16


def test_state_frame_has_documented_keys_and_nested_fallen(duck):
    reply = duck.call("robot.subscribe", {"hz": 50})
    assert reply["accepted"] is True
    state = wait_for_state(duck)
    assert set(state) == STATE_KEYS
    assert set(state["safety"]) == SAFETY_KEYS
    assert state["safety"]["fallen"] is False
    assert "fallen" not in state, "fallen must be nested under safety, not top level"
    assert len(state["joints"]) == 14 and len(state["targets"]) == 14   # this model has no mouth servo
    assert len(state["head"]) == 4
    assert state["policy"] in {"walk", "stand", "sit"}
    assert state["safety"]["gravity"][2] < -0.9, "upright: projected gravity points down"


def test_discrete_intents_answer_accepted(duck):
    assert duck.call("robot.stop") == {"accepted": True}
    look = duck.call("robot.look", {"x": 0.3, "y": 0.0, "z": -0.12})
    assert set(look) == {"head", "clamped"}
    assert set(look["head"]) == {"neck_pitch", "head_pitch", "head_yaw", "head_roll"}
    do = duck.call("robot.do", {"skill": "kick_left"})
    assert set(do) >= {"accepted"}


def test_unknown_method_and_bad_params_fail_loudly(duck):
    with pytest.raises(RuntimeError, match="-32601"):
        duck.call("robot.enable", {"enabled": True})
    with pytest.raises(RuntimeError, match="-32602"):
        duck.call("robot.do", {"skill": "kick_lef"})
    with pytest.raises(RuntimeError, match="-32602"):
        duck.call("robot.move", {"vx": 0.3, "speed": 1})


def test_sim_frame_is_a_640x360_jpeg_when_gl_is_available(duck):
    try:
        reply = duck.call("sim.frame")
    except RuntimeError as e:
        pytest.skip(f"no OpenGL context here: {e}")
    assert (reply["width"], reply["height"], reply["format"]) == (640, 360, "jpeg")
    assert base64.b64decode(reply["data"])[:2] == b"\xff\xd8", "JPEG magic bytes"


def test_example_client_script_runs_against_the_server(server):
    out = subprocess.run(
        [sys.executable, "examples/client.py", "--socket", server.path, "--seconds", "0.5", "--vx", "0.3"],
        cwd=ROOT, capture_output=True, text=True, timeout=20,
    )
    assert out.returncode == 0, out.stderr
    assert "hello ->" in out.stdout and "stop -> {'accepted': True}" in out.stdout


# -- physics, real time ----------------------------------------------------------

def test_walking_moves_the_duck_forward(duck):
    duck.call("robot.subscribe", {"hz": 20})
    first = wait_for_state(duck)
    x0, t0 = first["odom"]["position"][0], first["t"]
    wall_limit = time.monotonic() + 10.0
    while duck.state()["t"] - t0 < 3.0 and time.monotonic() < wall_limit:   # 3 s of simulated time
        duck.notify("robot.move", {"vx": 0.3, "vy": 0.0, "vyaw": 0.0})       # resend, or the deadman stops it
        time.sleep(0.05)
    state = duck.state()
    assert state["t"] - t0 >= 3.0, f"only {state['t'] - t0:.1f} s simulated in 10 s: the loop is far below real time"
    assert state["policy"] == "walk"
    assert state["move"]["applied"][0] == pytest.approx(0.3)
    assert state["odom"]["position"][0] - x0 > 0.2, "0.3 m/s commanded for 3 s (measured ~0.12 m/s achieved)"
    assert state["safety"]["fallen"] is False


def test_move_expires_without_resend(duck):
    duck.call("robot.subscribe", {"hz": 20})
    duck.notify("robot.move", {"vx": 0.3})
    time.sleep(duckd_sim.MOVE_EXPIRY + 0.3)
    state = duck.state()
    assert state["move"]["applied"] == [0.0, 0.0, 0.0]
    assert state["move"]["limited_by"] == ["expired"]


def camera_look_z(sim) -> float:
    """z of the camera's look direction in the world: 0 level, negative is down."""
    with sim.lock:
        R = sim.data.cam_xmat[sim.cam].reshape(3, 3)
        return float(-R[2, 2])                            # MuJoCo cameras look along their own -z


def test_look_at_the_floor_turns_the_camera_down(duck, server):
    time.sleep(1.0)                                       # settle standing
    z_level = camera_look_z(server.sim)
    duck.call("robot.look", {"x": 0.3, "y": 0.0, "z": -0.12})
    time.sleep(1.5)
    z_down = camera_look_z(server.sim)
    assert z_down < z_level - 0.1, f"look z went {z_level:.2f} -> {z_down:.2f}, expected more negative"
