"""
Motors interface to handle robot.move commands.

Follows Academy's MotorsNode with the caveat that this interface
uses a background thread to prevent motor commands from expiring after 0.5s.
"""

import threading

from microduck_academy.hal_interfaces.client import DuckClient


class MotorsDuck:
    def __init__(self, client: DuckClient):
        # connected client
        self._duck = client

        # sendV and sendW only write the target speeds into
        # the shared _cmd (protected by _cmd_lock)
        self._cmd = {"vx": 0.0, "vy": 0.0, "vyaw": 0.0}
        self._cmd_lock = threading.Lock()

        # resend rate for robot.move; the server expires it after 0.5 s
        self._move_hz = 20

        # define a background thread for _move_loop:
        # the server drops a robot.move command after 0.5s, so
        # we use a thread to make _move_loop run in the background.
        # (To avoid clashes btw reading and writing to _cmd we use _cmd_lock)
        self._move_thread = threading.Thread(target=self._move_loop, daemon=True)

        # add an event to stop the motors loop
        self._move_stopped = threading.Event()

        # start background thread for _move_loop
        # (must be last, to read everything above)
        self._move_thread.start()

    def _move_loop(self) -> None:
        while not self._move_stopped.is_set():
            # read command dict from a quick snapshot
            with self._cmd_lock:
                command_params = dict(self._cmd)

            # send robot.move call
            try:
                self._duck.notify("robot.move", command_params)
            except OSError:
                # catches OSError from .notify when connection closes
                return
            self._move_stopped.wait(
                1 / self._move_hz
            )  # returns immediately if event is true

    def sendV(self, v: float) -> None:
        """Forward speed, m/s. Below about 0.25 the walker stands still."""
        with self._cmd_lock:
            self._cmd["vx"] = float(v)

    def sendW(self, w: float) -> None:
        """Yaw rate, rad/s, positive turns left."""
        with self._cmd_lock:
            self._cmd["vyaw"] = float(w)

    def stop(self) -> None:
        with self._cmd_lock:
            self._cmd.update(vx=0.0, vy=0.0, vyaw=0.0)
        self._duck.call("robot.stop")

    def close(self) -> None:
        "Stops the move loop. The duck will keep the last command until the server expires it."
        self._move_stopped.set()
