"""
Odometry interface to format robot.state as Pose3d.

Follows Academy's OdometryNode. The interface is handed an already connected client.
"""

from dataclasses import dataclass

from microduck_academy.hal_interfaces.client import DuckClient


@dataclass
class Pose3d:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0
    timeStamp: float = 0.0


class OdometryDuck:
    def __init__(self, client: DuckClient):
        self._duck = client

    def getPose3d(self) -> Pose3d:
        """Odometry in the frame the duck woke up in. Returns origin if not reported yet."""
        state = self._duck.state()
        odom = (state or {}).get("odom")

        if odom is None:
            return Pose3d()
        else:
            x, y, z = odom["position"]
            return Pose3d(x=x, y=y, z=z, yaw=odom["yaw"], timeStamp=state["t"])