"""JSON-RPC 2.0 over a Unix socket, one object per line. The wire half of docs/api.md.

Generic over a `handler` with two methods, and it knows nothing about ducks:

    handler.handle(method: str, params) -> result        # raise RpcError to answer an error
    handler.state() -> dict                              # one state frame for the stream

Rules implemented here, from the real robot's protocol: a message without `id`
is a notification and never gets a reply, not even an error. One method (the
`stream_method`, `robot.subscribe` for the duck) turns the connection into a
stream of `stream_notification` messages carrying handler.state() at the
requested `hz`; calling it again on the same connection changes the rate.
"""
from __future__ import annotations

import json
import logging
import os
import socket
import threading
import time
from typing import Any, Callable

log = logging.getLogger("duckd")

PARSE_ERROR = -32700
METHOD_NOT_FOUND = -32601
INVALID_PARAMS = -32602
INTERNAL_ERROR = -32603


class RpcError(Exception):
    """Raise from a handler to answer a JSON-RPC error with this code and message."""

    def __init__(self, code: int, message: str):
        super().__init__(message)
        self.code = code


def _line(obj: dict) -> bytes:
    return (json.dumps(obj) + "\n").encode("utf-8")


class Connection:
    """One client: its socket, a send lock (the stream thread and the reply path share it), its stream rate."""

    def __init__(self, sock: socket.socket):
        self.sock = sock
        self.send_lock = threading.Lock()
        self.stream_hz: float | None = None      # None until the client subscribes
        self.open = True

    def send(self, obj: dict) -> bool:
        with self.send_lock:
            try:
                self.sock.sendall(_line(obj))
                return True
            except OSError:
                return False

    def reply(self, msg_id: Any, result: Any) -> None:
        self.send({"jsonrpc": "2.0", "id": msg_id, "result": result})

    def error(self, msg_id: Any, code: int, message: str) -> None:
        self.send({"jsonrpc": "2.0", "id": msg_id, "error": {"code": code, "message": message}})


class JsonRpcServer:
    """Listen on a Unix socket and serve `handler`. Use as a context manager or start()/close()."""

    def __init__(self, path: str, handler: Any, default_hz: float = 50.0,
                 stream_method: str = "robot.subscribe", stream_notification: str = "robot.state"):
        self.path = path
        self.handler = handler
        self.default_hz = default_hz
        self.stream_method = stream_method
        self.stream_notification = stream_notification
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []

    # -- lifecycle ----------------------------------------------------------------
    def start(self) -> "JsonRpcServer":
        if os.path.exists(self.path):
            os.unlink(self.path)
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.bind(self.path)
        self.sock.listen()
        self.sock.settimeout(0.2)                # so the accept loop notices close()
        self._spawn(self._accept_loop)
        log.info("listening on %s", self.path)
        return self

    def close(self) -> None:
        self._stop.set()
        for t in self._threads:
            t.join(timeout=1.0)
        self.sock.close()
        if os.path.exists(self.path):
            os.unlink(self.path)

    def __enter__(self) -> "JsonRpcServer":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.close()

    def _spawn(self, target: Callable, *args) -> None:
        t = threading.Thread(target=target, args=args, daemon=True)
        t.start()
        self._threads.append(t)

    # -- one thread per connection, plus one for its stream ---------------------------
    def _accept_loop(self) -> None:
        while not self._stop.is_set():
            try:
                sock, _ = self.sock.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            self._spawn(self._serve, Connection(sock))

    def _serve(self, conn: Connection) -> None:
        try:
            with conn.sock, conn.sock.makefile("r", encoding="utf-8") as reader:
                for raw in reader:
                    if self._stop.is_set():
                        return
                    self._dispatch(conn, raw)
        finally:
            conn.open = False                    # stops the stream thread

    def _dispatch(self, conn: Connection, raw: str) -> None:
        try:
            msg = json.loads(raw)
        except json.JSONDecodeError:
            conn.error(None, PARSE_ERROR, "parse error")
            return
        msg_id, method, params = msg.get("id"), msg.get("method"), msg.get("params")
        try:
            result = self.handler.handle(method, params)
        except RpcError as e:
            if msg_id is not None:               # notifications never get a reply, not even an error
                conn.error(msg_id, e.code, str(e))
            return
        except Exception as e:                   # a bug in the handler must not kill the connection silently
            log.exception("handler failed on %s", method)
            if msg_id is not None:
                conn.error(msg_id, INTERNAL_ERROR, f"{type(e).__name__}: {e}")
            return
        if method == self.stream_method:
            self._start_or_retune_stream(conn, params)
        if msg_id is not None:
            conn.reply(msg_id, result)

    def _start_or_retune_stream(self, conn: Connection, params: Any) -> None:
        hz = float((params or {}).get("hz") or self.default_hz)
        first_time = conn.stream_hz is None
        conn.stream_hz = hz
        if first_time:
            self._spawn(self._stream, conn)

    def _stream(self, conn: Connection) -> None:
        while conn.open and not self._stop.is_set():
            if not conn.send({"jsonrpc": "2.0", "method": self.stream_notification, "params": self.handler.state()}):
                return
            time.sleep(1.0 / conn.stream_hz)
