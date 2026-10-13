"""Demo usage of the hardware interfaces for the duck server.

Connects to the Unix socket, says hello, subscribes to robot.state, walks
forward for a few seconds through MotorsDuck, printing the pose from
OdometryDuck, then stops.

MotorsDuck resends robot.move at 20 Hz in a background thread, so the loop
below only sets the target speed once. For the raw JSON messages, see
docs/api.md.

Start duckd_sim.py first, then:
    uv run python examples/demo.py 

To run for a specific socket, vx or length of time:
    uv run python examples/demo.py --socket /tmp/duckd.sock --vx 0.3 --seconds 5
"""

import argparse
import time

from microduck_academy.hal_interfaces.client import API_VERSION, DuckClient
from microduck_academy.hal_interfaces.motors import MotorsDuck
from microduck_academy.hal_interfaces.odometry import OdometryDuck


def main() -> None:
    # args parser
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--socket", default="/tmp/duckd.sock")
    ap.add_argument(
        "--vx",
        type=float,
        default=0.3,
        help="m/s; below ~0.25 the walker does not move",
    )
    ap.add_argument("--seconds", type=float, default=5.0)
    args = ap.parse_args()

    # instantiate client and connect
    duck = DuckClient(args.socket)
    print("hello ->", duck.call("hello", {"api_version": API_VERSION}))
    print("subscribe ->", duck.call("robot.subscribe", {"hz": 10}))

    # instantiate hardware interfaces
    motors = MotorsDuck(duck)
    odometry = OdometryDuck(duck)

    # walk forward for a few seconds
    motors.sendV(args.vx)
    t_end = time.monotonic() + args.seconds
    while time.monotonic() < t_end:
        pose = odometry.getPose3d()
        print(f"t={pose.timeStamp:.2f} x={pose.x:.2f} y={pose.y:.2f} yaw={pose.yaw:.2f}")
        time.sleep(0.1)

    # stop and close motors
    motors.stop()
    motors.close()

    # close connection
    duck.close()


if __name__ == "__main__":
    main()
