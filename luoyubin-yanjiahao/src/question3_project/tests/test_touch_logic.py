"""题三触碰判定与输入校验回归测试，运行：python3 -m pytest src/question3_project/tests

不依赖 Isaac Sim，覆盖：
- 判定口径：只有力学证据或 PhysX 接触报告才算触碰，距离不参与（对应 H127）；
- 输入校验：未知玩具、缺位姿字段、位姿含非有限数值都要报行号。
"""
import json
import sys
from pathlib import Path

import pytest

PROJECT = Path(__file__).resolve().parents[1]
ENTRY = PROJECT.parents[1]
sys.path.insert(0, str(PROJECT))

import touch_toy  # noqa: E402

INPUT = ENTRY / "inputs" / "q3_demo_input.jsonl"
RECORD = json.loads(INPUT.read_text(encoding="utf-8").splitlines()[0])
PHYSX_EVENT = {"actors": ["/World/abr_machine/robot_root/gripper/gripper_left_link", "/World/company/toys/Rubik"]}


def test_decide_touch_requires_evidence():
    """有力矩/回升/位移或 PhysX 报告才 PASS；两者都没有时即使距离很近也 FAIL。"""
    assert touch_toy.decide_touch(True, None) is True
    assert touch_toy.decide_touch(False, PHYSX_EVENT) is True
    assert touch_toy.decide_touch(False, None) is False


def test_decide_touch_matches_contact_field():
    """答案里的 contact_detected 必须与 PASS 判定同源，避免运行 PASS 而自检判失败。"""
    for contact in (True, False):
        for event in (None, PHYSX_EVENT):
            assert touch_toy.decide_touch(contact, event) == (contact or event is not None)


def test_load_records_accepts_demo():
    """官方 demo 输入三行全部通过校验。"""
    records = touch_toy.load_records(INPUT)
    assert [record["name"] for record in records] == ["rubik_cube", "dinosaur", "soft_toy"]


def test_load_records_rejects_unknown_toy(tmp_path):
    path = tmp_path / "bad.jsonl"
    path.write_text(json.dumps({**RECORD, "name": "unicorn"}) + "\n", encoding="utf-8")
    with pytest.raises(ValueError, match=r"第 1 行玩具名未知"):
        touch_toy.load_records(path)


def test_load_records_rejects_missing_pose(tmp_path):
    broken = {key: value for key, value in RECORD.items() if key != "scene_object_in_world"}
    path = tmp_path / "bad.jsonl"
    path.write_text(json.dumps(broken) + "\n", encoding="utf-8")
    with pytest.raises(ValueError, match=r"第 1 行缺少字段 'scene_object_in_world'"):
        touch_toy.load_records(path)


@pytest.mark.parametrize("bad", [float("nan"), float("inf")])
def test_load_records_rejects_non_finite_pose(tmp_path, bad):
    broken = json.loads(json.dumps(RECORD))
    broken["base_footprint_in_world"]["rpy"][1] = bad
    path = tmp_path / "bad.jsonl"
    path.write_text(json.dumps(broken) + "\n", encoding="utf-8")
    with pytest.raises(ValueError, match=r"第 1 行.*含非有限数值"):
        touch_toy.load_records(path)
