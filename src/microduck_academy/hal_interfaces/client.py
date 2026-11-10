"""Client for the duck server, used by every class in hal_interfaces.

The connection to the duck server: sends requests and notifications,
matches replies by id, keeps the latest robot.state. It plays the role
rclpy (the ROS 2 Python library) plays in the Academy's hal_interfaces.
"""

import json
import os
import socket
import threading
import time

API_VERSION = 16  # duck-ipc-proto lib.rs line 164, commit 590b986

DEFAULT_SOCKET = "/tmp/duckd.sock"


class DuckClient:
    """Connection to the duck server over a Unix socket.

    Protocol: JSON-RPC 2.0, one JSON object per line. A *request* carries
    an id and gets a reply with the same id (see call()). A *notification*
    has no id and gets no reply (see notify()); the server also sends
    notifications, such as robot.state.
    """

    def __init__(self, path: str | None = None):
        path = path or os.environ.get("DUCK_SOCKET", DEFAULT_SOCKET)
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            self.sock.connect(path)
        # if no socket, or socket is stale (no server behind)
        except (FileNotFoundError, ConnectionRefusedError):
            raise SystemExit(
                f"no server listening at {path}: start duckd_sim.py first "
                "(the socket file is missing, or left over from a server that stopped)"
            ) from None
        self.reader = self.sock.makefile("r", encoding="utf-8")
        self.next_id = 1
        self.replies: dict[int, dict] = {}
        self.last_state: dict | None = None
        self._connected = True

        # set up locks
        self.lock = threading.Lock()  # for next_id, replies and last_state
        self._send_lock = threading.Lock()  # to _send

        threading.Thread(target=self._read_loop, daemon=True).start()

    def _send(self, msg: dict) -> None:
        """Send one message as a single JSON line."""
        line = (json.dumps(msg) + "\n").encode("utf-8")
        with self._send_lock:
            try:
                self.sock.sendall(line)
            except OSError:
                # report any send failure as ConnectionError, so students see one error type
                self._connected = False
                raise ConnectionError("lost connection to the duck server") from None

    def notify(self, method: str, params: dict | None = None) -> None:
        msg = {"jsonrpc": "2.0", "method": method}
        if params is not None:
            msg["params"] = params
        self._send(msg)

    def call(
        self, method: str, params: dict | None = None, timeout: float = 2.0
    ) -> dict:
        # get message id from next_id
        with self.lock:
            msg_id = self.next_id
            self.next_id += 1

        # send message
        msg = {"jsonrpc": "2.0", "id": msg_id, "method": method}
        if params is not None:
            msg["params"] = params
        self._send(msg)

        # retrieve reply with timeout
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.lock:
                if (reply := self.replies.pop(msg_id, None)) is not None:
                    if "error" in reply:
                        raise RuntimeError(f"{method}: {reply['error']}")
                    return reply.get("result")
                self._check_connected()  # fail fast if connection is down
            time.sleep(0.005)
        raise TimeoutError(f"no reply to {method} (id {msg_id}) within {timeout}s")

    def _read_loop(self) -> None:
        """Reader thread: hand every incoming line to _handle until the connection ends."""
        try:
            for line in self.reader:
                self._handle(json.loads(line))
        except OSError:
            # silence OSErrors (state() and call() already report the lost connection)
            pass
        finally:
            self._connected = False

    def _handle(self, msg: dict) -> None:
        """Keep a reply under its id, or the latest robot.state; ignore other notifications.

        Replies are matched by id; notifications have no id. To react to
        another notification, add a branch here.
        """
        with self.lock:
            if "id" in msg:
                self.replies[msg["id"]] = msg
            elif msg.get("method") == "robot.state":
                self.last_state = msg["params"]

    def _check_connected(self) -> None:
        """Raises ConnectionError if the reader thread has stopped.

        Call while holding self.lock
        """
        if not self._connected:
            raise ConnectionError("lost connection to the duck server")

    def state(self) -> dict | None:
        with self.lock:
            self._check_connected()
            return self.last_state

    def wait_for_state(self, timeout: float = 2.0) -> dict:
        """Block until the first robot.state arrives.

        Subscribe first.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if (state := self.state()) is not None:
                return state
            time.sleep(0.005)

        raise TimeoutError(
            f"no state received within {timeout}s; did you call robot.subscribe?"
        )

    def close(self) -> None:
        """Close the socket; the server then stops streaming to us."""
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.sock.close()
