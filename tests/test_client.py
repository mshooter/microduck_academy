"""DuckClient notices when the server connection drops."""

import os
import shutil
import socket
import tempfile

import pytest
from microduck_academy.hal_interfaces.client import DuckClient


@pytest.fixture
def listener():
    """Listening socket (no data)"""
    # Define unique tmp path for socket
    folder = tempfile.mkdtemp(prefix="duck-", dir="/tmp")
    path = os.path.join(folder, "duck.sock")

    # Make a socket and bind it to path
    sckt = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sckt.bind(path)

    # Start accepting connections (1 client wait in line)
    sckt.listen(1)

    yield sckt

    # Teardown: close the socket and remove folder with socket file
    sckt.close()
    shutil.rmtree(folder, ignore_errors=True)


def test_client_connecting_to_stale_socket(listener):
    """Connecting to a stale socket raises and shows hint."""
    # Get path listener bound to
    path = listener.getsockname()

    # Close socket: file stays but no-one is listening
    listener.close()

    # ConnectionRefusedError should be caught
    with pytest.raises(SystemExit, match="start duckd_sim.py"):
        DuckClient(path)