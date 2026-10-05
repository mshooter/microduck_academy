"""Demo client for the duck server. Standard library only.

Connects to the Unix socket, says hello, subscribes to robot.state, sends
robot.move at 20 Hz for a few seconds, then robot.stop. Every message is one
JSON object on one line; see docs/api.md.

Deliberately wire-level: no setV()/getImage() here, those belong in HAL.py.

Cannot run until duckd_sim.py exists; it is that server's first test.

    uv run python examples/client.py --socket /tmp/duckd.sock --vx 0.3 --seconds 5
"""

import argparse
import time

from microduck_academy.hal_interfaces.client import API_VERSION, DuckClient


def main() -> None:
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

    duck = DuckClient(args.socket)
    print("hello ->", duck.call("hello", {"api_version": API_VERSION}))
    print("subscribe ->", duck.call("robot.subscribe", {"hz": 10}))

    # Continuous intent: resend at 20 Hz or the server lets it expire.
    t_end = time.monotonic() + args.seconds
    while time.monotonic() < t_end:
        duck.notify("robot.move", {"vx": args.vx, "vy": 0.0, "vyaw": 0.0})
        s = duck.state()
        if s is not None:
            x, y, _ = s["odom"]["position"]
            print(
                f"t={s['t']:.2f} policy={s['policy']} fallen={s['safety']['fallen']} x={x:.2f} y={y:.2f}"
            )
        time.sleep(0.05)

    print("stop ->", duck.call("robot.stop"))


if __name__ == "__main__":
    main()
