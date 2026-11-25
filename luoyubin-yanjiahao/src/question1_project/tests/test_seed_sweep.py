"""题一 0~127 全种子回归：每颗种子的答案都要过一遍完整校验器。

单点冒烟只能证明某颗种子没写崩；这里把 128 颗种子全部跑完，
再用 check_output.py 的正式校验口径（五段顺序、10 关节、限位、
阶段隔离、手指 mimic、最终态）逐行验收，防止 seed 分支退化。

运行：python3 -m pytest src/question1_project/tests
"""
import json
import sys
from pathlib import Path

import pytest

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))

import check_output  # noqa: E402
import q1_driver  # noqa: E402

LIMITS = json.loads((PROJECT / "config" / "joint_limits.json").read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def mimic():
    return check_output.load_mimic(check_output.resolve_urdf(None, None))


def _record(seed):
    return {"id": f"seed-{seed}", "step": "all", "seed": seed, "move_sec": 0.0, "pause_sec": 0.0}


def test_all_seeds_pass_official_checks(mimic):
    """0~127 每颗种子的答案都必须通过题一正式校验。"""
    for seed in range(128):
        record = _record(seed)
        result = q1_driver.run_record(record, LIMITS, q1_driver.NullDriver(), paced=False)
        problems = check_output.check_record(result, record, LIMITS, mimic)
        assert not problems, f"seed {seed} 未通过：{problems}"


def test_seeds_actually_vary(mimic):
    """128 颗种子不能退化成同一个目标。"""
    arm_targets = {
        tuple(sorted(q1_driver.plan_targets(seed).items()))
        for seed in range(128)
    }
    assert len(arm_targets) >= 120, f"只有 {len(arm_targets)} 种不同的目标组合"


def test_seed_seven_matches_demo_shape(mimic):
    """官方 demo 用 seed 7，单跑一次也要通过校验。"""
    record = _record(7)
    result = q1_driver.run_record(record, LIMITS, q1_driver.NullDriver(), paced=False)
    assert result["final_joint_state"] == result["keyframes"][-1]["joint_state"]
    assert not check_output.check_record(result, record, LIMITS, mimic)
