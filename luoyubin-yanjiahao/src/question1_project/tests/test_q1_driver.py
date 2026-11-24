"""题一驱动的单元测试，运行：python3 -m pytest src/question1_project/tests"""
import json
import sys
from pathlib import Path

import pytest

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))

import q1_driver  # noqa: E402

LIMITS = json.loads((PROJECT / "config" / "joint_limits.json").read_text(encoding="utf-8"))
RECORD = {"id": "t", "step": "all", "seed": 7, "move_sec": 0.0, "pause_sec": 0.0}


def test_plan_targets_same_seed():
    """同 seed 结果一致。"""
    assert q1_driver.plan_targets(7) == q1_driver.plan_targets(7)


def test_plan_targets_diff_seed():
    """不同 seed 结果不同。"""
    assert q1_driver.plan_targets(7) != q1_driver.plan_targets(8)


def test_plan_targets_within_limits():
    """采样结果全部在限位内。"""
    for seed in range(200):
        state = {**q1_driver.initial_state(), **q1_driver.plan_targets(seed)}
        q1_driver.check_limits(state, LIMITS)


def test_check_limits_rejects_bad_state():
    """越界、未知关节、缺关节都必须报错。"""
    with pytest.raises(ValueError, match="超出限位"):
        q1_driver.check_limits({**q1_driver.initial_state(), "lift_joint": 9.9}, LIMITS)
    with pytest.raises(ValueError, match="未知关节"):
        q1_driver.check_limits({**q1_driver.initial_state(), "joint9": 0.0}, LIMITS)
    with pytest.raises(ValueError, match="缺少关节"):
        partial = q1_driver.initial_state()
        partial.pop("joint3")
        q1_driver.check_limits(partial, LIMITS)


def test_run_record_structure():
    """五段顺序、每帧 10 关节、末帧与 final 一致、手指开合值正确。"""
    result = q1_driver.run_record(RECORD, LIMITS, q1_driver.NullDriver(), paced=False)
    names = [frame["name"] for frame in result["keyframes"]]
    assert names == ["lift", "head", "arm", "gripper_open", "gripper_close"]
    assert result["id"] == "t" and result["step"] == "all"
    for frame in result["keyframes"]:
        assert sorted(frame["joint_state"]) == sorted(q1_driver.JOINTS)
    assert result["final_joint_state"] == result["keyframes"][-1]["joint_state"]

    open_state = result["keyframes"][3]["joint_state"]
    close_state = result["keyframes"][4]["joint_state"]
    assert open_state["arm_finger_joint1"] == LIMITS["arm_finger_joint1"]["upper"]
    assert close_state["arm_finger_joint1"] == 0.0
    assert close_state["joint1"] == open_state["joint1"]


def test_load_records_reports_line(tmp_path):
    """输入错误时报错信息包含行号与字段名。"""
    bad = tmp_path / "bad.jsonl"
    bad.write_text('{"id": "a", "step": "all"}\n', encoding="utf-8")
    with pytest.raises(ValueError, match="第 1 行缺少字段 'move_sec'"):
        q1_driver.load_records(bad)

    broken = tmp_path / "broken.jsonl"
    broken.write_text("{oops}\n", encoding="utf-8")
    with pytest.raises(ValueError, match="第 1 行不是合法 JSON"):
        q1_driver.load_records(broken)

    negative = tmp_path / "neg.jsonl"
    negative.write_text(json.dumps({**RECORD, "move_sec": -1}) + "\n", encoding="utf-8")
    with pytest.raises(ValueError, match="不能为负"):
        q1_driver.load_records(negative)
