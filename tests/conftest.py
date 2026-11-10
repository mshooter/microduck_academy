import contextlib
import io
import os
import shutil
import tempfile
import time

import pytest
from microduck_academy import duckd_sim


@pytest.fixture(scope="module")
def server():
    # A short path on purpose: Unix socket paths are limited to ~104 bytes on macOS,
    # and pytest's tmp_path there is longer than that.
    folder = tempfile.mkdtemp(prefix="duck-", dir="/tmp")
    path = os.path.join(folder, "duck.sock")
    try:
        with contextlib.redirect_stdout(
            io.StringIO()
        ):  # Pollen's loop prints on every command
            with duckd_sim.DuckServer(path) as s:
                yield s
    finally:
        shutil.rmtree(folder, ignore_errors=True)


@pytest.fixture
def wait_sim_seconds():
    """Return a function that waits until the server's own clock has advanced `seconds`.

    The deadman and the policy run on sim time, which falls behind wall time on a slow
    CI runner, so sleeping wall time is flaky. Takes a subscribed DuckClient and
    returns the latest robot.state.

    Helper exposed as a fixture so the test modules can share it easily.
    """

    def wait(duck, seconds, wall_limit=10.0):
        t0 = duck.wait_for_state()["t"]
        deadline = time.monotonic() + wall_limit
        while duck.state()["t"] - t0 < seconds:
            assert (
                time.monotonic() < deadline
            ), f"only {duck.state()['t'] - t0:.2f} s simulated in {wall_limit} s"
            time.sleep(0.02)
        return duck.state()

    return wait
