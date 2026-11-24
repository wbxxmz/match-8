"""题三规划回归测试，运行：python3 -m pytest src/question3_project/tests

不依赖 Isaac Sim，覆盖：
- 关节链与完整 10 关节校验；
- 底盘/玩具/objects_link 位姿变换闭环；
- 答案位姿与最终关节 FK 一致；
- 三个玩具中心的 IK 残差；
- 一组不同初值的 FK 与限位检查。
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

PROJECT = Path(__file__).resolve().parents[1]
ENTRY = PROJECT.parents[1]
sys.path.insert(0, str(PROJECT))

import planner  # noqa: E402


def load_jsonl(path):
    return [json.loads(line) for line in Path(path).read_text(encoding="utf-8").splitlines() if line.strip()]


RECORDS = load_jsonl(ENTRY / "inputs/q3_demo_input.jsonl")
ANSWERS = load_jsonl(ENTRY / "answers/q3_demo_output.jsonl")

# 与 demo 不同的一组初值：升降 8 厘米、肘部展开、腕部内收，全部在限位内。
OTHER_STATE = {
    "lift_joint": 0.08,
    "pitch_joint": 0.0,
    "joint1": 0.3,
    "joint2": 0.5,
    "joint3": 1.1,
    "joint4": -0.2,
    "joint5": -0.8,
    "joint6": 0.1,
    "arm_finger_joint1": 0.4,
    "arm_finger_joint2": 0.4,
}


@pytest.fixture(scope="module")
def chain():
    return planner.load_robot_chain()


@pytest.fixture(scope="module")
def limits():
    return planner.load_limits()


def test_chain_order(chain):
    """链从底盘出发，终点是 objects_link，六个臂关节顺序固定。"""
    names = [joint.name for joint in chain]
    assert names[0] == "base_joint"
    assert names[-1] == "object_joint"
    assert [name for name in names if name.startswith("joint")][-6:] == [
        "joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
    ]
    assert "pitch_joint" not in names
    assert "arm_finger_joint1" not in names


def test_full_state_rejects_missing():
    """少一个关节必须报错，不能静默补零。"""
    partial = {name: 0.0 for name in planner.JOINTS if name != "joint4"}
    with pytest.raises(ValueError):
        planner.full_state(partial)


@pytest.mark.parametrize("record", RECORDS, ids=[record["id"] for record in RECORDS])
def test_transform_roundtrip(record):
    """底盘×objects_link×玩具位姿应回到玩具在底盘系的位姿。"""
    toy_in_base = planner.toy_transform_in_base(
        record["base_footprint_in_world"], record["scene_object_in_world"]
    )
    answer = next(item for item in ANSWERS if item["id"] == record["id"])
    objects_in_base = planner.make_transform(
        answer["objects_link_in_base_footprint"]["xyz"], answer["objects_link_in_base_footprint"]["rpy"]
    )
    toy_in_objects = planner.toy_pose_in_objects(objects_in_base, toy_in_base)
    reconstructed = objects_in_base @ toy_in_objects
    assert np.linalg.norm(reconstructed[:3, 3] - toy_in_base[:3, 3]) < 1e-9
    assert np.linalg.norm(reconstructed[:3, :3] - toy_in_base[:3, :3]) < 1e-9


@pytest.mark.parametrize("answer", ANSWERS, ids=[answer["id"] for answer in ANSWERS])
def test_answer_pose_matches_fk(chain, answer):
    """答案里的 objects_link 位姿必须能由最终关节重算出来。"""
    fk = planner.fk_objects_link(chain, answer["final_joint_state"])
    pose = answer["objects_link_in_base_footprint"]
    expected = planner.make_transform(pose["xyz"], pose["rpy"])
    assert np.linalg.norm(fk[:3, 3] - expected[:3, 3]) < 1e-5
    assert np.linalg.norm(fk[:3, :3] - expected[:3, :3]) < 1e-4


@pytest.mark.parametrize("record", RECORDS, ids=[record["id"] for record in RECORDS])
def test_ik_reaches_toy_center(chain, limits, record):
    """从玩具方位出发的 IK 应把 objects_link 解到玩具中心，残差在容差内。"""
    toy_in_base = planner.toy_transform_in_base(
        record["base_footprint_in_world"], record["scene_object_in_world"]
    )
    target_xyz = toy_in_base[:3, 3]
    rotation = planner.approach_rotation(target_xyz, tilt_deg=50.0)
    seed = planner.reach_seed(math.atan2(target_xyz[1], target_xyz[0]))
    solution, position_error, rotation_error = planner.solve_ik(
        chain, target_xyz, rotation, seed, limits
    )
    state = dict(solution)
    state.update(planner.CHAIN_FILLER)
    fk = planner.fk_objects_link(chain, state)
    assert np.linalg.norm(fk[:3, 3] - target_xyz) < 1e-4
    assert position_error < 1e-4
    assert rotation_error < 1e-3


@pytest.mark.parametrize("answer", ANSWERS, ids=[answer["id"] for answer in ANSWERS])
def test_answer_joints_within_limits(limits, answer):
    state = answer["final_joint_state"]
    assert set(state) == set(planner.JOINTS)
    for name, value in state.items():
        lower, upper = limits[name]
        assert lower <= value <= upper, f"{name}={value} 越界 [{lower}, {upper}]"


def test_alternative_initial_state(chain, limits):
    """换一组初值：FK 结果有限、objects_link 在地面以上、所有关节在限位内。"""
    for name, value in OTHER_STATE.items():
        lower, upper = limits[name]
        assert lower <= value <= upper
    transform = planner.fk_objects_link(chain, OTHER_STATE)
    assert np.isfinite(transform).all()
    assert 0.0 < transform[2, 3] < 1.5
    assert planner.link_origins_below_floor(chain, OTHER_STATE) == []
