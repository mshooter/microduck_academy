"""The shipped policies walk in our loop, and the head camera renders (when a GL context exists)."""
import io
import contextlib

import mujoco
import pytest

from microduck_academy import smoke


@pytest.fixture(scope="module")
def walked():
    """One 5 s walk, shared by the tests below (takes about two seconds)."""
    with contextlib.redirect_stdout(io.StringIO()): 
        return smoke.run(seconds=5.0, vx=smoke.DEFAULT_VX, frame_path=None)


def test_duck_walks_forward(walked):
    # Measured 0.63 m on 2026-09-26 with mujoco 3.10.0; 0.4 leaves margin for platform noise.
    assert walked["forward_m"] > 0.4
    assert walked["policy"] == "walking"


def test_duck_stays_upright(walked):
    assert walked["upright"]


def test_render_fails_fast_after_the_first_gl_failure(monkeypatch):
    """Retrying GL after GLFW failed to initialise hangs on headless Linux; the second call must not retry."""
    calls = []

    def broken_renderer(*args, **kwargs):
        calls.append(1)
        raise RuntimeError("no display")

    monkeypatch.setattr(smoke, "_gl_error", None)
    monkeypatch.setattr(smoke.mujoco, "Renderer", broken_renderer)
    for _ in range(2):
        with pytest.raises(RuntimeError):
            smoke.render_frame(model=None, data=None)
    assert len(calls) == 1, "the renderer must be tried once, then the first error reused"


def test_head_camera_renders_forward_and_upright():
    with contextlib.redirect_stdout(io.StringIO()):
        model = smoke.load_model()
        data = mujoco.MjData(model)
        policy = smoke.build_policy(model, data)
        smoke.reset_pose(model, data, policy)
    cam = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, smoke.CAMERA)
    R = data.cam_xmat[cam].reshape(3, 3)
    look, up = -R[:, 2], R[:, 1]
    assert look[0] > 0.99, "camera must look along +x (forward) at the reset pose"
    assert up[2] > 0.99, "camera must be upright (+z up), not rolled a quarter turn"
    try:
        frame = smoke.render_frame(model, data)
    except Exception as e:          # no EGL/GLX/CGL context on this machine (headless CI)
        pytest.skip(f"no OpenGL context for offscreen rendering: {e}")
    assert frame.shape == (smoke.FRAME_H, smoke.FRAME_W, 3)
    assert frame.std() > 10, "a flat image means the camera is looking into geometry"
