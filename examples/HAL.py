"""Draft Robotics Academy HAL for the Microduck. One layer above client.py.

Students call plain functions (setV, setW, getImage, getPose3d, ...) the way
other Academy exercises do; this module turns them into docs/api.md messages.

robot.move is a continuous intent: the server forgets it after 0.5 s. So
setV()/setW() only store the wanted velocity, and a background thread resends
it at 20 Hz. Everything else is a discrete request, answered once.

    uv run python -m microduck_academy.duckd_sim     # terminal 1
    uv run python examples/HAL.py                    # terminal 2: short demo
"""
import base64
import io
import threading
import time
from dataclasses import dataclass

import numpy as np
from PIL import Image

from examples.client import API_VERSION, DuckClient
from microduck_academy.duckd_sim import DEFAULT_SOCKET

MOVE_HZ = 20  # resend rate for robot.move; the server expires it after 0.5 s

_duck: DuckClient | None = None
_cmd = {"vx": 0.0, "vy": 0.0, "vyaw": 0.0}
_cmd_lock = threading.Lock()


@dataclass
class Pose3d:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0


def _client() -> DuckClient:
    """Connect on first use, so `import HAL` alone does not need the server."""
    global _duck
    if _duck is None:
        _duck = DuckClient(DEFAULT_SOCKET)
        _duck.call("hello", {"api_version": API_VERSION})
        _duck.call("robot.subscribe", {"hz": 50})
        threading.Thread(target=_move_loop, daemon=True).start()
    return _duck


def _move_loop() -> None:
     # We use a loop in a separate thread so that students don't need to keep calling setV
    # (in line with ROS robots and other examples)
    while True:
        with _cmd_lock:
            params = dict(_cmd)
        _duck.notify("robot.move", params)
        time.sleep(1 / MOVE_HZ)


def _state() -> dict:
    return _client().state() or {}


# --- motors ---------------------------------------------------------------
# Q: why these don't do client.notify?
def setV(v: float) -> None:
    """Forward speed, m/s. Below about 0.25 the walker stands still."""
    _client()
    with _cmd_lock:
        _cmd["vx"] = float(v)


def setW(w: float) -> None:
    """Yaw rate, rad/s, positive turns left."""
    _client()
    with _cmd_lock:
        _cmd["vyaw"] = float(w)


def stop() -> None:
    with _cmd_lock:
        _cmd.update(vx=0.0, vy=0.0, vyaw=0.0)
    _client().call("robot.stop")


# --- head and skills ------------------------------------------------------

def lookAt(x: float, y: float, z: float) -> bool:
    """Point the camera at (x, y, z) in the trunk frame. False if clamped."""
    result = _client().call("robot.look", {"x": x, "y": y, "z": z})
    return not result["clamped"]


def doSkill(skill: str) -> bool:
    """ground_pick, kick_left, kick_right, sit_toggle or roulade.

    True means the skill started, not that it finished; see getPolicy().
    """
    return _client().call("robot.do", {"skill": skill})["accepted"]


# --- sensors --------------------------------------------------------------

def getImage() -> np.ndarray:
    """Head camera, 360x640x3 BGR uint8, like Academy's other getImage()."""
    frame = _client().call("sim.frame")  # sim only; hardware needs another source
    rgb = np.asarray(Image.open(io.BytesIO(base64.b64decode(frame["data"]))).convert("RGB"))
    return rgb[:, :, ::-1].copy()


def getPose3d() -> Pose3d:
    """Odometry in the frame the duck woke up in. Origin if not reported yet."""
    odom = _state().get("odom")
    if odom is None:
        return Pose3d()
    x, y, z = odom["position"]
    return Pose3d(x, y, z, odom["yaw"])


def isFallen() -> bool:
    return _state().get("safety", {}).get("fallen", False)


def getPolicy() -> str | None:
    """walk, stand, sit, or the name of the skill that is running."""
    return _state().get("policy")


# --- sim only -------------------------------------------------------------

def reset() -> None:
    with _cmd_lock:
        _cmd.update(vx=0.0, vy=0.0, vyaw=0.0)
    _client().call("sim.reset")


def spawnBall(x: float, y: float) -> None:
    _client().call("sim.spawnBall", {"x": x, "y": y})


if __name__ == "__main__":
    setV(0.3)
    for _ in range(50):  # 5 s
        if isFallen():
            break
        print(getPose3d())
        time.sleep(0.1)
    stop()
    print("image", getImage().shape)
