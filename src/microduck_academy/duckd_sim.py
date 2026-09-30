"""The duck server: MuJoCo plus Pollen's shipped policies, speaking docs/api.md on a Unix socket.

    uv run python -m microduck_academy.duckd_sim                 # headless, /tmp/duckd.sock, real time
    uv run python -m microduck_academy.duckd_sim --viewer        # plus a MuJoCo window (macOS: mjpython)
    uv run python examples/client.py                             # in another terminal

The simulation is the smoke test's loop (load_model, build_policy, control_tick,
step_physics) run at 50 Hz in a thread, wrapped in a lock. Socket handling is
in rpc.py. This file is the mapping between the protocol and Pollen's
PolicyInference:

    robot.move       -> set_vel_cmd, guarded by a Deadman that zeroes it MOVE_EXPIRY s after the last move
    robot.stop       -> set_vel_cmd(0, 0, 0)
    robot.head       -> head_offset, a command the policy follows (offsets from its default head pose)
    robot.look       -> head_offset from a geometric approximation with measured signs
    robot.do         -> trigger_ground_pick / trigger_behavior / toggle_sit
    robot.subscribe  -> streaming is done by rpc.py; the frame comes from DuckSim.state()
    sim.frame        -> render_frame (needs an OpenGL context; EGL/GLX/CGL)
    sim.reset        -> reset_pose
    sim.spawnBall    -> move the ball's free joint

Threads, and the one lock they share:

    client ◄══ /tmp/duckd.sock ══► rpc.py JsonRpcServer
                                    ├─ accept loop ─► one thread per client: read a line,
                                    │                 handler.handle(method, params), reply if it had an id
                                    └─ per subscribed client: stream thread, handler.state() every 1/hz

    DuckSim (handler)  ─── everything below holds self.lock ───
      handle()  METHODS table -> pokes PolicyInference (set_vel_cmd, head_offset, trigger_*)
      state()   trunk pose, joints, gravity, deadman -> one robot.state frame
      frame     copies MjData under the lock, then renders the copy outside it
      tick()    control thread, 50 Hz real time:
                  1. deadman.check(t): zero the velocity if the last move is older than MOVE_EXPIRY
                  2. smoke.control_tick(policy): observation -> ONNX -> data.ctrl
                  3. smoke.step_physics(): 4 x mj_step
      --viewer  main thread, view.sync() under the same lock

To add a method: write a `_name(self, p)` method on DuckSim that returns the
result (or raises RpcError), add it to DuckSim.METHODS, add a section to
docs/api.md, add a test to tests/test_contract.py.

Pollen's loop prints on every command; stdout is sent to /dev/null unless --verbose.
The server logs to stderr.
"""
from __future__ import annotations

import argparse
import base64
import io
import logging
import math
import os
import sys
import threading
import time
from typing import Any, Callable

import mujoco

from . import smoke
from .rpc import INTERNAL_ERROR, INVALID_PARAMS, METHOD_NOT_FOUND, JsonRpcServer, RpcError

log = logging.getLogger("duckd")

API_VERSION = 16                 # duck-ipc-proto lib.rs, commit 590b986
DEFAULT_SOCKET = "/tmp/duckd.sock"
CONTROL_HZ = 1.0 / smoke.CONTROL_DT
MOVE_EXPIRY = 0.5                # s of sim time without a robot.move before the velocity is zeroed. Real value unknown.
HEAD_FIELDS = ("neck_pitch", "head_pitch", "head_yaw", "head_roll")
HEAD_LIMIT = 1.4                 # rad, PolicyInference.head_max for the new-command policies
BALL_RADIUS = 0.035              # m, from assets/microduck/ball.xml
CAMERA_ABOVE_TRUNK = 0.05        # m, rough camera height above the trunk origin, for robot.look
# Measured 2026-09-26 by commanding +0.5 rad and reading the camera's look vector:
# +head_pitch looks DOWN, +head_yaw looks LEFT, neck_pitch barely moves the gaze.
LOOK_PITCH_GAIN = 1.3            # commanded offset per radian of wanted gaze change (0.5 rad gave ~0.37 rad)
SKILLS = ("ground_pick", "kick_left", "kick_right", "sit_toggle", "roulade")
POLICY_NAMES = {"walking": "walk", "standing": "stand", "sit": "sit"}   # Pollen's names -> the protocol's

Params = dict[str, Any]


def _params(params: Any, allowed: set[str]) -> Params:
    """The params object of a request, or an INVALID_PARAMS error naming any field we do not know."""
    p = params if isinstance(params, dict) else {}
    if unknown := set(p) - allowed:
        raise RpcError(INVALID_PARAMS, f"unknown field(s) {sorted(unknown)}; allowed {sorted(allowed)}")
    return p


def _clip(v: float, limit: float) -> float:
    return max(-limit, min(limit, v))


class Deadman:
    """The rule for continuous intents: a robot.move is honoured for MOVE_EXPIRY seconds of sim time,
    then the velocity is zeroed until the next one. `requested` is what the client last sent,
    `applied()` is what the policy is being given right now; robot.state reports both."""

    def __init__(self) -> None:
        self.requested = [0.0, 0.0, 0.0]
        self.armed_at: float | None = None
        self.expired = False

    def arm(self, twist: list[float], now: float) -> None:
        self.requested = list(twist)
        self.armed_at = now
        self.expired = False

    def check(self, now: float) -> bool:
        """True exactly once, on the tick the move expires."""
        if self.armed_at is None or self.expired or now - self.armed_at <= MOVE_EXPIRY:
            return False
        self.expired = True
        return True

    def applied(self) -> list[float]:
        return [0.0, 0.0, 0.0] if self.expired else list(self.requested)

    def limited_by(self) -> list[str]:
        return ["expired"] if self.expired and any(self.requested) else []


class DuckSim:
    """The simulated robot. Everything touching MuJoCo or the policy holds self.lock."""

    def __init__(self) -> None:
        self.lock = threading.RLock()
        self.model = smoke.load_model()
        self.data = mujoco.MjData(self.model)
        self.policy = smoke.build_policy(self.model, self.data)
        self.cam = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, smoke.CAMERA)
        # Joint names in actuator order, which is also the order of the policy's qpos indices,
        # so the head's slots are looked up by name rather than assumed.
        self.joint_names = [mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT,
                                              int(self.model.actuator_trnid[i, 0])) for i in range(self.model.nu)]
        self.head_slots = [self.joint_names.index(name) for name in HEAD_FIELDS]
        self.deadman = Deadman()
        self._frame_data = mujoco.MjData(self.model)
        self._frame_lock = threading.Lock()
        self.loop_hz = 0.0
        self.missed = 0
        self.reset()

    # -- simulation -----------------------------------------------------------
    def reset(self) -> None:
        with self.lock:
            self._end_any_skill()
            smoke.reset_pose(self.model, self.data, self.policy)
            self.policy.head_offset[:] = 0.0
            self.deadman = Deadman()

    def tick(self) -> None:
        """One 50 Hz step: deadman, policy, physics."""
        with self.lock:
            if self.deadman.check(self.data.time):
                self.policy.set_vel_cmd(0.0, 0.0, 0.0)
            smoke.control_tick(self.policy)
            smoke.step_physics(self.model, self.data)

    def run_forever(self, stop: threading.Event, realtime: bool = True) -> None:
        """Tick until `stop` is set, pacing to real time and counting the ticks that overran."""
        period = smoke.CONTROL_DT
        window_start, window_ticks = time.monotonic(), 0
        next_tick = time.monotonic()
        while not stop.is_set():
            self.tick()
            window_ticks += 1
            next_tick += period
            now = time.monotonic()
            if now - window_start >= 1.0:
                self.loop_hz = window_ticks / (now - window_start)
                window_start, window_ticks = now, 0
            if realtime:
                if now - next_tick > period:
                    self.missed += 1
                    next_tick = now
                else:
                    time.sleep(max(0.0, next_tick - now))

    # -- the only place that reaches into PolicyInference's internals ------------------
    # Pollen's methods print and return None on refusal, so acceptance is read from the
    # mode flags they set. Check here first when the vendored file is updated.
    def _end_any_skill(self) -> None:
        pol = self.policy
        if pol.behavior_mode is not None:            # a kick or roll in progress: hand control back
            pol._end_behavior()
        if pol.ground_pick_mode:
            pol._end_ground_pick()
        if pol.sit_mode:
            pol.toggle_sit()

    def _skill_holding(self) -> str | None:
        pol = self.policy
        return pol.behavior_mode or ("ground_pick" if pol.ground_pick_mode else None)

    def _start_skill(self, skill: str) -> bool:
        pol = self.policy
        if skill == "ground_pick":
            pol.trigger_ground_pick()
            return pol.ground_pick_mode
        if skill == "sit_toggle":
            was = pol.sit_mode
            pol.toggle_sit()
            return pol.sit_mode != was
        pol.trigger_behavior(skill)
        return pol.behavior_mode == skill

    def _set_head(self, head: list[float]) -> None:
        self.policy.head_offset[:] = head
        self.policy._update_command()               # push the new head command into the observation

    def _policy_name(self) -> str:
        name = self.policy.behavior_mode or self.policy.current_policy
        return POLICY_NAMES.get(name, name)

    # -- observation ----------------------------------------------------------
    def state(self) -> dict[str, Any]:
        """One robot.state frame, the shape in docs/api.md section 2."""
        with self.lock:
            x, y, z, yaw = smoke.trunk_pose(self.policy)
            q = self.data.qpos[self.policy.joint_qpos_indices]
            gravity = self.policy.get_projected_gravity()
            move: dict[str, Any] = {"requested": list(self.deadman.requested), "applied": self.deadman.applied()}
            if self.deadman.limited_by():
                move["limited_by"] = self.deadman.limited_by()
            return {
                "t": round(float(self.data.time), 3),
                "move": move,
                "head": [round(float(q[i]), 4) for i in self.head_slots],
                "policy": self._policy_name(),
                "safety": {"fallen": bool(z < smoke.FALL_Z), "limp": False,
                           "gravity": [round(float(g), 3) for g in gravity]},
                "loop": {"hz": round(self.loop_hz, 1), "missed": self.missed},
                "joints": [round(float(v), 4) for v in q],
                "targets": [round(float(v), 4) for v in self.data.ctrl],
                "odom": {"position": [round(x, 4), round(y, 4), round(z, 4)], "yaw": round(yaw, 4)},
            }

    def frame_jpeg(self) -> bytes:
        """Render the head camera from a snapshot. The sim lock is held only to copy the state
        (microseconds), not while rendering and encoding, so a slow frame never stalls the 50 Hz
        loop. `_frame_lock` keeps two clients from rendering into the same snapshot at once."""
        from PIL import Image
        with self._frame_lock:
            with self.lock:
                mujoco.mj_copyData(self._frame_data, self.model, self.data)
            try:
                rgb = smoke.render_frame(self.model, self._frame_data)
            except Exception as e:  # no OpenGL context (CI, a bare server)
                raise RpcError(INTERNAL_ERROR, f"cannot render: {type(e).__name__}: {e}") from None
        buf = io.BytesIO()
        Image.fromarray(rgb).save(buf, format="JPEG", quality=85)
        return buf.getvalue()

    # -- protocol methods, one per entry in METHODS --------------------------------
    def handle(self, method: str, params: Any) -> Any:
        if (fn := self.METHODS.get(method)) is None:
            raise RpcError(METHOD_NOT_FOUND, f"method not found: {method}")
        return fn(self, params)

    def _hello(self, params: Any) -> Params:
        _params(params, {"api_version"})
        return {"api_version": API_VERSION, "daemon_version": None, "revision": None}

    def _move(self, params: Any) -> None:
        p = _params(params, {"vx", "vy", "vyaw"})
        twist = [float(p.get(k, 0.0)) for k in ("vx", "vy", "vyaw")]
        with self.lock:
            self.deadman.arm(twist, self.data.time)
            if self._skill_holding() is None:        # a running skill owns the policy until it ends
                self.policy.set_vel_cmd(*twist)

    def _stop(self, params: Any) -> Params:
        _params(params, set())
        with self.lock:
            self.deadman.arm([0.0, 0.0, 0.0], self.data.time)
            self.policy.set_vel_cmd(0.0, 0.0, 0.0)
        return {"accepted": True}

    def _head(self, params: Any) -> None:
        p = _params(params, set(HEAD_FIELDS))
        head = [_clip(float(p.get(k, 0.0)), HEAD_LIMIT) for k in HEAD_FIELDS]
        with self.lock:
            self._set_head(head)

    def _look(self, params: Any) -> Params:
        p = _params(params, {"x", "y", "z", "neck_pitch"})
        x, y, z = (float(p.get(k, 0.0)) for k in ("x", "y", "z"))
        dist = math.hypot(x, y)
        if dist < 1e-6:
            raise RpcError(INVALID_PARAMS, "x and y cannot both be zero")
        down = math.atan2(CAMERA_ABOVE_TRUNK - z, dist)      # wanted gaze below level, rad
        pitch = LOOK_PITCH_GAIN * down
        yaw = math.atan2(y, x)                                # +yaw looks left (measured)
        head = {"neck_pitch": float(p.get("neck_pitch", 0.0)),
                "head_pitch": round(_clip(pitch, HEAD_LIMIT), 4),
                "head_yaw": round(_clip(yaw, HEAD_LIMIT), 4),
                "head_roll": 0.0}
        with self.lock:
            self._set_head([head[k] for k in HEAD_FIELDS])
        return {"head": head, "clamped": abs(pitch) > HEAD_LIMIT or abs(yaw) > HEAD_LIMIT}

    def _do(self, params: Any) -> Params:
        p = _params(params, {"skill"})
        skill = p.get("skill")
        if skill not in SKILLS:
            raise RpcError(INVALID_PARAMS, f"unknown skill {skill!r}; one of {list(SKILLS)}")
        with self.lock:
            holding = self._skill_holding()
            if holding and skill != "sit_toggle":
                return {"accepted": False, "reason": f"{holding} is holding the robot"}
            ok = self._start_skill(skill)
        return {"accepted": True} if ok else {"accepted": False, "reason": f"{skill} refused by the policy loop"}

    def _subscribe(self, params: Any) -> Params:
        _params(params, {"hz"})                              # the stream itself is started by rpc.py
        return {"accepted": True, **smoke.POLICY_ROLES}

    def _frame(self, params: Any) -> Params:
        _params(params, set())
        return {"width": smoke.FRAME_W, "height": smoke.FRAME_H, "format": "jpeg",
                "data": base64.b64encode(self.frame_jpeg()).decode("ascii")}

    def _reset(self, params: Any) -> Params:
        _params(params, set())
        self.reset()
        return {"accepted": True}

    def _spawn_ball(self, params: Any) -> Params:
        p = _params(params, {"x", "y"})
        if self.policy.ball_qpos_adr is None:
            return {"accepted": False, "reason": "no ball in this scene"}
        bx, by = float(p.get("x", 0.3)), float(p.get("y", 0.0))
        with self.lock:
            adr, vadr = self.policy.ball_qpos_adr, self.policy.ball_qvel_adr
            self.data.qpos[adr:adr + 3] = [bx, by, BALL_RADIUS]  # resting on the floor
            self.data.qpos[adr + 3:adr + 7] = [1, 0, 0, 0]
            self.data.qvel[vadr:vadr + 6] = 0.0
            mujoco.mj_forward(self.model, self.data)
        return {"accepted": True}

    METHODS: dict[str, Callable[["DuckSim", Any], Any]] = {
        "hello": _hello,
        "robot.move": _move,
        "robot.stop": _stop,
        "robot.head": _head,
        "robot.look": _look,
        "robot.do": _do,
        "robot.subscribe": _subscribe,
        "sim.frame": _frame,
        "sim.reset": _reset,
        "sim.spawnBall": _spawn_ball,
    }


# -- running it -------------------------------------------------------------------

class DuckServer:
    """DuckSim + its control thread + the socket. Context manager for tests and main()."""

    def __init__(self, path: str = DEFAULT_SOCKET, realtime: bool = True):
        self.sim = DuckSim()
        self.rpc = JsonRpcServer(path, self.sim, default_hz=CONTROL_HZ)
        self.realtime = realtime
        self._stop = threading.Event()

    @property
    def path(self) -> str:
        return self.rpc.path

    def start(self) -> "DuckServer":
        self._thread = threading.Thread(target=self.sim.run_forever, args=(self._stop, self.realtime), daemon=True)
        self._thread.start()
        self.rpc.start()
        return self

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        self.rpc.close()

    def __enter__(self) -> "DuckServer":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.close()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--socket", default=DEFAULT_SOCKET, help="Unix socket path to listen on")
    ap.add_argument("--viewer", action="store_true", help="show a MuJoCo window (macOS: run with mjpython)")
    ap.add_argument("--verbose", action="store_true", help="let the policy loop print to stdout")
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s", stream=sys.stderr)
    if not args.verbose:
        sys.stdout = open(os.devnull, "w")       # Pollen's loop prints on every command; we cannot edit it

    with DuckServer(args.socket) as duck:
        log.info("duck standing, %d joints, socket %s; Ctrl-C to stop", duck.sim.model.nu, args.socket)
        try:
            if args.viewer:
                import mujoco.viewer
                # launch_passive runs mj_forward on the shared MjData; the control thread
                # must not step it at the same time, so create the viewer under the lock too.
                with duck.sim.lock:
                    view = mujoco.viewer.launch_passive(duck.sim.model, duck.sim.data,
                                                        show_left_ui=False, show_right_ui=False)
                with view:
                    while view.is_running():
                        with duck.sim.lock:
                            view.sync()
                        time.sleep(smoke.CONTROL_DT)
            else:
                while True:
                    time.sleep(1.0)
        except KeyboardInterrupt:
            log.info("bye")


if __name__ == "__main__":
    main()
