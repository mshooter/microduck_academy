"""Smallest complete client for the duck server. Standard library only.

Connects to the Unix socket, says hello, subscribes to robot.state, sends
robot.move at 20 Hz for a few seconds, then robot.stop. Every message is one
JSON object on one line; see docs/api.md.

Deliberately wire-level: no setV()/getImage() here, those belong in HAL.py.

Cannot run until duckd_sim.py exists; it is that server's first test.

    uv run python examples/client.py --socket /tmp/duckd.sock --vx 0.3 --seconds 5
"""
import argparse
import json
import socket
import threading
import time

API_VERSION = 16  # duck-ipc-proto lib.rs line 164, commit 590b986


class DuckClient:
    def __init__(self, path: str):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            self.sock.connect(path)
        except FileNotFoundError:
            raise SystemExit(
                f"no server listening at {path}: a Unix socket is a file the server "
                "creates when it starts, so start duckd_sim.py first"
            ) from None
        self.reader = self.sock.makefile("r", encoding="utf-8")
        self.next_id = 1
        self.replies: dict[int, dict] = {}
        self.last_state: dict | None = None
        self.lock = threading.Lock()
        threading.Thread(target=self._read_loop, daemon=True).start()

    # One JSON object per line. A request carries an id; a notification does not.
    def _send(self, msg: dict) -> None:
        self.sock.sendall((json.dumps(msg) + "\n").encode("utf-8"))

    def notify(self, method: str, params: dict | None = None) -> None:
        msg = {"jsonrpc": "2.0", "method": method}
        if params is not None:
            msg["params"] = params
        self._send(msg)

    def call(self, method: str, params: dict | None = None, timeout: float = 2.0) -> dict:
        with self.lock:
            msg_id = self.next_id
            self.next_id += 1
        msg = {"jsonrpc": "2.0", "id": msg_id, "method": method}
        if params is not None:
            msg["params"] = params
        self._send(msg)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.lock:
                if (reply := self.replies.pop(msg_id, None)) is not None:
                    if "error" in reply:
                        raise RuntimeError(f"{method}: {reply['error']}")
                    return reply.get("result")
            time.sleep(0.005)
        raise TimeoutError(f"no reply to {method} (id {msg_id}) within {timeout}s")

    # Replies are matched by id, not by order; notifications have no id.
    def _read_loop(self) -> None:
        for line in self.reader:
            msg = json.loads(line)
            if "id" in msg:
                with self.lock:
                    self.replies[msg["id"]] = msg
            elif msg.get("method") == "robot.state":
                with self.lock:
                    self.last_state = msg["params"]

    def state(self) -> dict | None:
        with self.lock:
            return self.last_state

    def close(self) -> None:
        """Close the socket; the server then stops streaming to us."""
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.sock.close()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--socket", default="/tmp/duckd.sock")
    ap.add_argument("--vx", type=float, default=0.3, help="m/s; below ~0.25 the walker does not move")
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
            print(f"t={s['t']:.2f} policy={s['policy']} fallen={s['safety']['fallen']} x={x:.2f} y={y:.2f}")
        time.sleep(0.05)

    print("stop ->", duck.call("robot.stop"))


if __name__ == "__main__":
    main()
