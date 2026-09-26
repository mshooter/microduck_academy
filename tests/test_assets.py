"""The vendored model and policies are complete and have the shapes the server relies on."""
import re
from pathlib import Path

import mujoco
import onnxruntime as ort
import pytest

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "assets" / "microduck" / "scene_ball.xml"
ROBOT = ROOT / "assets" / "microduck" / "robot_allcollisions.xml"
POLICIES = sorted((ROOT / "assets" / "policies").glob("*.onnx"))


def test_scene_loads_with_expected_structure():
    m = mujoco.MjModel.from_xml_path(str(SCENE))
    assert m.nu == 14, "the duck has 14 actuated joints"
    cameras = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_CAMERA, i) for i in range(m.ncam)]
    assert "head_camera" in cameras


def test_every_referenced_mesh_is_vendored():
    referenced = set(re.findall(r'file="([^"]+\.stl)"', ROBOT.read_text()))
    assert len(referenced) == 38
    missing = [f for f in referenced if not (ROOT / "assets" / "microduck" / "assets" / f).exists()]
    assert missing == []


def test_nine_policies_present():
    assert [p.name for p in POLICIES] == [
        "alpha_ground_pick.onnx", "alpha_sitstand.onnx", "alpha_stand.onnx",
        "alpha_walking.onnx", "ball_kick_left.onnx", "ball_kick_right.onnx",
        "roller.onnx", "roller_crouch.onnx", "roulade.onnx",
    ]


@pytest.mark.parametrize("policy", POLICIES, ids=lambda p: p.stem)
def test_policy_io_shapes(policy):
    s = ort.InferenceSession(str(policy), providers=["CPUExecutionProvider"])
    (inp,) = s.get_inputs()
    (out,) = s.get_outputs()
    assert (inp.name, inp.shape) == ("obs", [1, 61])
    assert (out.name, out.shape) == ("actions", [1, 14])
