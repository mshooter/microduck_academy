"""Smoke test: stand the duck up, walk it forward, save one head-camera frame.

    uv run python -m microduck_academy.smoke             # headless; prints the result, writes out/frame.jpg
    uv run python -m microduck_academy.smoke --viewer    # same, in a MuJoCo window (macOS: use mjpython)

This is the smallest thing that proves the shipped policies walk outside Pollen's training
repo and that a camera image can be rendered without a screen. 
"""
import argparse
import math
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from .vendor.infer_policy import PolicyInference

# --- setup -------------------------------------------------------------------
# TODO: Works for an editable install (uv sync). If the package is ever installed non-editable,
# the assets must move into the package; this is a later problem. 
ROOT = Path(__file__).resolve().parents[2]
ASSETS = ROOT / "assets"
SCENE = ASSETS / "microduck" / "scene_ball.xml"
POLICIES = ASSETS / "policies"

# --- numbers the policies were trained with ---------------------------------------------
PHYSICS_DT = 0.005          # duck_playground.py overrides the XML's 0.002 to this
DECIMATION = 4              # one policy call per 4 physics steps: 50 Hz
CONTROL_DT = PHYSICS_DT * DECIMATION
# XL330 servos saturate at ~1.75 A. Torque = kt * I with kt = 0.36601 from Pollen's `bam`
# motor model (xl330, m6). Computed 2026-09-26; hardcoded so we do not depend on `bam`.
TORQUE_LIMIT_NM = 0.6405
FALL_Z = 0.08               # trunk lower than this means the duck fell (duck_bench.py)

# --- camera ------------------------------------------------------------------------------
CAMERA = "head_camera"
FRAME_W, FRAME_H = 640, 360     # the real duck's default stream (robotd-params Quality::Q360p30)
# Pollen's MJCF places `head_camera` with the CAD frame: as exported it looks BACKWARDS into the
# head shell and is rolled a quarter turn (the real sensor is mounted that way too; mediad leaves
# the rotation to the consumer). MuJoCo cameras look along their own -z with +y up. This is the
# orientation, in the head body's frame, that looks forward and upright. Derived 2026-09-26 by
# expressing (right=-y, up=+z, look=+x) in the world and converting with the body's rotation.
CAMERA_QUAT_FORWARD = (0.7071068, 0.0, 0.0, -0.7071068)

POLICY_ROLES = {
    "walking_onnx_path": "alpha_walking.onnx",
    "standing_onnx_path": "alpha_stand.onnx",
    "sitstand_onnx_path": "alpha_sitstand.onnx",
    "ground_pick_onnx_path": "alpha_ground_pick.onnx",
    "kick_left_onnx_path": "ball_kick_left.onnx",
    "kick_right_onnx_path": "ball_kick_right.onnx",
    "roulade_onnx_path": "roulade.onnx",
}


def load_model() -> mujoco.MjModel:
    """The scene with the training timestep and the real servo's torque cap."""
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    model.opt.timestep = PHYSICS_DT
    model.actuator_forcerange[:, 0] = -TORQUE_LIMIT_NM
    model.actuator_forcerange[:, 1] = TORQUE_LIMIT_NM
    model.actuator_forcelimited[:] = 1
    cam = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, CAMERA)
    model.cam_quat[cam] = CAMERA_QUAT_FORWARD
    return model


def build_policy(model, data) -> PolicyInference:
    """Pollen's loop around the policies, with all shipped roles loaded."""
    paths = {role: str(POLICIES / name) for role, name in POLICY_ROLES.items()}
    return PolicyInference(model, data, new_cmd_obs=True, use_projected_gravity=True, **paths)


def reset_pose(model, data, policy) -> None:
    """Stand the duck at the origin in its default pose, standing policy active."""
    mujoco.mj_resetData(model, data)
    adr = policy._trunk_qpos_adr
    data.qpos[adr:adr + 3] = [0.0, 0.0, 0.125]      # trunk x, y, z
    data.qpos[adr + 3:adr + 7] = [1, 0, 0, 0]       # no rotation
    for i, qi in enumerate(policy.joint_qpos_indices):
        data.qpos[qi] = policy.default_pose[i]
    data.ctrl[:] = policy.default_pose
    policy.last_action[:] = 0
    policy.set_vel_cmd(0.0, 0.0, 0.0)
    mujoco.mj_forward(model, data)


# --- the two halves of the loop, kept apart on purpose (this is what Gazebo replaces) ---

def control_tick(policy) -> None:
    """One 50 Hz step: advance timed behaviours, run the policy, write motor targets."""
    policy.update_ground_pick_phase(CONTROL_DT)
    policy.update_behavior(CONTROL_DT)
    policy.apply_action(policy.infer())


def step_physics(model, data, n: int = DECIMATION) -> None:
    for _ in range(n):
        mujoco.mj_step(model, data)


# --- reading the robot ------------------------------------------------------------------

def trunk_pose(policy):
    """x, y, z in metres and yaw in radians, from the trunk's free joint."""
    q = policy.data.qpos
    adr = policy._trunk_qpos_adr
    x, y, z = (float(v) for v in q[adr:adr + 3])
    qw, qx, qy, qz = q[adr + 3:adr + 7]
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return x, y, z, yaw


def render_frame(model, data, width: int = FRAME_W, height: int = FRAME_H) -> np.ndarray:
    """RGB image from the head camera, no window needed. Needs an OpenGL context (EGL/GLX/CGL)."""
    renderer = mujoco.Renderer(model, height=height, width=width)
    try:
        renderer.update_scene(data, camera=CAMERA)
        return renderer.render().copy()
    finally:
        renderer.close()


# --- the smoke test itself --------------------------------------------------------------

# The shipped walker does not move for |vx| <= 0.2 (measured 2026-09-26: 0.1 and 0.2 give 1 cm in
# 5 s, 0.3 gives 62 cm). Pollen's own bench reports ~0.13 m/s achieved at 0.3 commanded.
DEFAULT_VX = 0.3


def run(seconds: float = 5.0, vx: float = DEFAULT_VX, settle: float = 1.0,
        frame_path: str | None = "out/frame.jpg", viewer: bool = False) -> dict:
    """Stand for `settle` s, walk forward at `vx` m/s for `seconds`, report and save a frame."""
    model = load_model()
    data = mujoco.MjData(model)
    policy = build_policy(model, data)
    reset_pose(model, data, policy)

    ticks_settle = int(round(settle / CONTROL_DT))
    ticks_walk = int(round(seconds / CONTROL_DT))
    x0, y0, _, _ = trunk_pose(policy)

    view = None
    if viewer:
        view = mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False)
    try:
        for tick in range(ticks_settle + ticks_walk):
            if tick == ticks_settle:
                policy.set_vel_cmd(vx, 0.0, 0.0)          # this flips PolicyInference to the walking policy
            control_tick(policy)
            step_physics(model, data)
            if view is not None:
                if not view.is_running():
                    break
                view.sync()
                time.sleep(CONTROL_DT)                     # real time, so the eye can follow
    finally:
        if view is not None:
            view.close()

    x, y, z, yaw = trunk_pose(policy)
    result = {
        "distance_m": math.hypot(x - x0, y - y0),
        "forward_m": x - x0,
        "sideways_m": y - y0,
        "yaw_deg": math.degrees(yaw),
        "upright": z > FALL_Z,
        "policy": policy.current_policy,
        "frame": None,
    }
    if frame_path:
        from PIL import Image
        Path(frame_path).parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(render_frame(model, data)).save(frame_path, quality=90)
        result["frame"] = str(Path(frame_path).resolve())
    return result


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--seconds", type=float, default=5.0, help="how long to walk")
    ap.add_argument("--vx", type=float, default=DEFAULT_VX, help="forward speed, m/s (below ~0.25 the walker does not move)")
    ap.add_argument("--out", default="out/frame.jpg", help="where to save the head-camera frame")
    ap.add_argument("--no-frame", action="store_true", help="skip rendering (no OpenGL needed)")
    ap.add_argument("--viewer", action="store_true", help="show a MuJoCo window and run in real time")
    args = ap.parse_args()
    r = run(args.seconds, args.vx, frame_path=None if args.no_frame else args.out, viewer=args.viewer)
    print(f"walked {r['distance_m']:.2f} m (forward {r['forward_m']:.2f}, sideways {r['sideways_m']:+.2f}), "
          f"heading {r['yaw_deg']:+.1f} deg, upright: {r['upright']}, policy: {r['policy']}")
    if r["frame"]:
        print(f"frame: {r['frame']}")


if __name__ == "__main__":
    main()
