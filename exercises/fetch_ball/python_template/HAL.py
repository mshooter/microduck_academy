"""Hardware abstraction layer.

Student-facing robot API. It connects to the already running server when imported.
"""

from microduck_academy.hal_interfaces.camera import CameraDuck
from microduck_academy.hal_interfaces.client import API_VERSION, DuckClient
from microduck_academy.hal_interfaces.motors import MotorsDuck
from microduck_academy.hal_interfaces.odometry import OdometryDuck

# Instantiate client
duck = DuckClient()
duck.call("hello", {"api_version": API_VERSION})
duck.call("robot.subscribe", {"hz": 50})
duck.wait_for_state()  # raises if nothing arrives

# Build interfaces
odometry = OdometryDuck(duck)
motors = MotorsDuck(duck)
camera = CameraDuck(duck)


# ------- Sensors and actuators common to many robots go through a hal_interfaces class -------
def setV(v):
    motors.sendV(float(v))


def setW(w):
    motors.sendW(float(w))


def stop():
    motors.stop()


def getImage():
    return camera.getImage()


def getPose3d():
    return odometry.getPose3d()


# ------- Duck-specific functions are written directly into each exercise's own HAL.py -------


def isFallen() -> bool:
    return duck.state()["safety"]["fallen"]


def lookAt(x: float, y: float, z: float) -> bool:
    """Point the camera at (x, y, z) in the trunk frame. False if clamped."""
    result = duck.call("robot.look", {"x": x, "y": y, "z": z})
    return not result["clamped"]


def doSkill(skill: str) -> bool:
    """ground_pick, kick_left, kick_right, sit_toggle or roulade.

    True means the skill started, not that it finished; see getPolicy().
    """
    return duck.call("robot.do", {"skill": skill})["accepted"]


def getPolicy() -> str | None:
    """walk, stand, sit, or the name of the skill that is running."""
    return (duck.state() or {}).get("policy")
