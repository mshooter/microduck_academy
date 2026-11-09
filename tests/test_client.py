"""DuckClient notices when the server connection drops."""

import json
import os
import shutil
import socket
import tempfile
import threading
import time

import pytest
from microduck_academy.hal_interfaces.client import DuckClient


@pytest.fixture
def listening_skt():
    """The listening socket of a basic server (no data)"""
    # Define unique tmp path for socket
    folder = tempfile.mkdtemp(prefix="duck-", dir="/tmp")
    path = os.path.join(folder, "duck.sock")

    # Make a socket and bind it to path
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.bind(path)

    # listen(1) lets one connection wait until accept() is called
    sock.listen(1)

    yield sock

    # Teardown: close the socket and remove folder with socket file
    sock.close()
    shutil.rmtree(folder, ignore_errors=True)

@pytest.fixture
def duck(listening_skt):
    """Start a duck client that connects to the minimal fake server.
    
    connect() succeeds before listening_socket accepts connections 
    because listening_socket accepts one waiting connection.
    """
    d = DuckClient(listening_skt.getsockname())
    yield d
    d.close()

def send(connection_socket, msg):
    """Push a message to the client.

    The input socket must be a connection one.
    """
    connection_socket.sendall(
        (json.dumps(msg) + "\n").encode("utf-8"),
    )


def test_client_raises_when_stale_socket(listening_skt):
    """Connecting to a stale socket raises and shows hint."""
    # Get path the listener bound to
    path = listening_skt.getsockname()

    # Close socket: file stays but no-one is listening
    listening_skt.close()

    # ConnectionRefusedError should be caught
    with pytest.raises(SystemExit, match="start duckd_sim.py"):
        DuckClient(path)


def test_state_raises_when_lost_connection(listening_skt, duck):
    """Check state() raises when connection is lost.
    
    accept() returns a new socket (the server's end of this client's
    connection); the test uses it to send data to the client.
    """
    # Accept the client's connection
    server_end_skt, _ = listening_skt.accept()

    # Send state
    send(
        server_end_skt,
        {"jsonrpc": "2.0", "method": "robot.state", "params": {"t": 1.0}},
    )

    # Check connection worked
    assert duck.wait_for_state() == {"t": 1.0}

    # Close server end of the connection
    server_end_skt.close()

    # ConnectionError should be raised
    # (rather than a stale state)
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        try:
            duck.state()
        except ConnectionError as e:
            assert "lost connection" in str(e)
            break
        time.sleep(0.01)
    else:
        pytest.fail("state() kept returning stale data after the server went away")


def test_call_raises_when_lost_connection(listening_skt, duck):
    """Check call() raises connection error rather than reply timeout."""
    # Accept client's connection
    server_end_skt, _ = listening_skt.accept()

    # Play server in another thread: reads request and die
    def read_request_then_close():
        # read bytes that arrive at the socket
        server_end_skt.recv(4096)  
        # close before replying
        server_end_skt.close()     
    threading.Thread(target=read_request_then_close).start()

    # call() should notice server has died
    with pytest.raises(ConnectionError, match="lost connection"):
        duck.call("hello", timeout=2.0)
