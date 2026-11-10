"""Tests for DuckClient's connection handling."""

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


def test_concurrent_sends(listening_skt, duck):
    """Two threads sending at once should produce whole JSON lines.

    Mimics the motor thread streaming robot.move while the main thread
    sends call()s. Large messages make sendall() write in chunks,
    so without the send lock the pieces of two messages could mix.
    """
    # Accept connection
    server_end_skt, _ = listening_skt.accept()

    # Start threads that send data to the server
    n_threads, n_msgs = 2, 300
    payload = "x" * 100_000

    def sender(thread_idx):
        for i in range(n_msgs):
            duck._send({"thread": thread_idx, "i": i, "payload": payload})

    threads = [threading.Thread(target=sender, args=(t,)) for t in range(n_threads)]
    for t in threads:
        t.start()

    # Read everything the client sends while the senders are still running.
    # The socket only holds a few hundred KB, so sendall() blocks until someone reads;
    # joining the threads first would hang the test.
    # Each message ends in one "\n", so there are exactly
    # n_threads * n_msgs lines even if messages get mixed.
    reader = server_end_skt.makefile("r", encoding="utf-8")
    lines = [reader.readline() for _ in range(n_threads * n_msgs)]

    # Wait for each of the senders to finish
    for t in threads:
        t.join()

    # Check every line is decodable and messages are not mixed
    broken = 0
    for line in lines:
        try:
            json.loads(line)
        except json.JSONDecodeError:
            broken += 1
    assert broken == 0, f"{broken} of {len(lines)} lines were garbled"


def test_handle_keeps_replies_and_latest_state(duck):
    """_handle stores a reply under its id and keeps only the newest robot.state."""
    duck._handle({"jsonrpc": "2.0", "id": 7, "result": {"accepted": True}})
    duck._handle({"jsonrpc": "2.0", "method": "robot.state", "params": {"t": 1.0}})
    duck._handle({"jsonrpc": "2.0", "method": "robot.state", "params": {"t": 2.0}})

    assert duck.replies[7]["result"] == {"accepted": True}
    assert duck.state() == {"t": 2.0}


def test_handle_ignores_other_notifications(duck):
    """Notifications other than robot.state change nothing."""
    duck._handle({"jsonrpc": "2.0", "method": "something.else", "params": {"x": 1}})

    assert duck.replies == {}
    assert duck.state() is None


def test_call_returns_the_reply_with_its_id(listening_skt, duck):
    """call() wakes up when the reply for its id arrives and returns its result."""
    server_end_skt, _ = listening_skt.accept()

    # Play server in another thread: read the request, answer with the same id
    def reply_to_request():
        request = json.loads(server_end_skt.makefile("r", encoding="utf-8").readline())
        send(
            server_end_skt,
            {"jsonrpc": "2.0", "id": request["id"], "result": {"api_version": 16}},
        )

    threading.Thread(target=reply_to_request).start()

    assert duck.call("hello", {"api_version": 16}, timeout=2.0) == {"api_version": 16}
    server_end_skt.close()


def test_call_times_out_when_no_reply(listening_skt, duck):
    """With the server connected but silent, call() raises TimeoutError, not ConnectionError."""
    server_end_skt, _ = listening_skt.accept()

    with pytest.raises(TimeoutError, match="no reply to hello"):
        duck.call("hello", timeout=0.2)
    server_end_skt.close()
