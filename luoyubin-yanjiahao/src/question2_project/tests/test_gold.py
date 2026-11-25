"""题二金标与自检，运行：python3 -m pytest src/question2_project/tests"""
import json
import sys
from pathlib import Path

import numpy as np
import pytest

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))

import fk  # noqa: E402
import run_q2  # noqa: E402
import transforms  # noqa: E402

ZERO_STATE = {name: 0.0 for name in fk.JOINTS}
GOLD_OBJECTS = {
    "xyz": [1.103715, -0.062685, 0.629136],
    "rpy": [0.0, 0.000832, 0.0],
    "quat_xyzw": [0.0, 0.000416, 0.0, 1.0],
}
GOLD_SCENE = {
    "xyz": [-0.603441, -0.437315, -0.329638],
    "rpy": [0.0, -0.000832, 0.0],
    "quat_xyzw": [0.0, -0.000416, 0.0, 1.0],
}
RECORD = {
    "id": "demo_zero",
    "joint_state": ZERO_STATE,
    "scene_object_in_base_footprint": {"xyz": [0.5, -0.5, 0.3], "rpy": [0.0, 0.0, 0.0]},
}


@pytest.fixture(scope="module")
def chain():
    return run_q2.load_chain(run_q2.find_urdf_root())


@pytest.fixture(scope="module")
def result(chain):
    return run_q2.run_record(RECORD, chain)


def test_chain_order(chain):
    """链必须从底盘出发，经过升降、机械臂，终点是 objects_link。"""
    names = [joint.name for joint in chain]
    assert names[0] == "base_joint"
    assert names[-1] == "object_joint"
    assert [name for name in names if name.startswith("joint")][-6:] == [
        "joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
    ]
    assert {"lift_joint", "arm_base_joint", "gripper_joint"}.issubset(names)
    # 头部与手指不在通往 objects_link 的链上。
    assert "pitch_joint" not in names
    assert "arm_finger_joint1" not in names


def test_gold_objects_link(result):
    """objects_link 与官方金标逐项比对，xyz 差 < 1e-4。"""
    actual = result["objects_link_in_base_footprint"]
    for axis, index in zip("xyz", range(3)):
        assert abs(actual["xyz"][index] - GOLD_OBJECTS["xyz"][index]) < 1e-4, axis
    assert actual["rpy"] == pytest.approx(GOLD_OBJECTS["rpy"], abs=1e-5)
    assert actual["quat_xyzw"] == pytest.approx(GOLD_OBJECTS["quat_xyzw"], abs=1e-5)


def test_gold_scene_object(result):
    """场景物体在夹爪系的位姿同样与金标一致。"""
    actual = result["scene_object_in_objects_link"]
    assert actual["xyz"] == pytest.approx(GOLD_SCENE["xyz"], abs=1e-4)
    assert actual["rpy"] == pytest.approx(GOLD_SCENE["rpy"], abs=1e-5)
    assert actual["quat_xyzw"] == pytest.approx(GOLD_SCENE["quat_xyzw"], abs=1e-5)


def test_inverse_round_trip(chain):
    """物体先正变换到夹爪系，再逆变换回底盘系，应回到原点。"""
    objects_in_base = fk.objects_link_transform(chain, ZERO_STATE)
    scene = transforms.make_transform(RECORD["scene_object_in_base_footprint"]["xyz"], [0, 0, 0])
    round_trip = objects_in_base @ transforms.invert_transform(objects_in_base) @ scene
    assert transforms.pose_from_transform(round_trip)["xyz"] == pytest.approx(
        RECORD["scene_object_in_base_footprint"]["xyz"], abs=1e-6
    )
    assert transforms.matrix_to_rpy(objects_in_base[:3, :3] @ objects_in_base[:3, :3].T) == pytest.approx(
        [0.0, 0.0, 0.0], abs=1e-6
    )


def test_unrelated_joints_do_not_change_output(chain):
    """头部与手指不在 objects_link 的链上，改动它们不能改变输出位姿。"""
    base = run_q2.run_record(RECORD, chain)
    for patch in (
        {"pitch_joint": 1.0},
        {"arm_finger_joint1": 0.5, "arm_finger_joint2": 0.5},
    ):
        record = json.loads(json.dumps(RECORD))
        record["joint_state"].update(patch)
        assert run_q2.run_record(record, chain) == base, patch


def test_lift_is_pure_z_translation(chain):
    """升降关节是移动关节，升高 0.2 米只能让位姿沿 z 平移，姿态不变。"""
    base = fk.objects_link_transform(chain, ZERO_STATE)
    lifted = fk.objects_link_transform(chain, {**ZERO_STATE, "lift_joint": 0.2})
    assert (lifted[:3, 3] - base[:3, 3]) == pytest.approx([0.0, 0.0, 0.2], abs=1e-9)
    assert lifted[:3, :3] == pytest.approx(base[:3, :3], abs=1e-12)


def test_joint6_changes_orientation_only(chain):
    """joint6 的旋转轴穿过 objects_link 原点，只改姿态不改位置。"""
    base = fk.objects_link_transform(chain, ZERO_STATE)
    turned = fk.objects_link_transform(chain, {**ZERO_STATE, "joint6": 1.0})
    assert turned[:3, 3] == pytest.approx(base[:3, 3], abs=1e-9)
    assert not np.allclose(turned[:3, :3], base[:3, :3])


def test_rpy_quat_consistency():
    """rpy 与 quat_xyzw 表示同一旋转，互转误差 < 1e-6。"""
    samples = [
        [0.0, 0.0, 0.0],
        [0.3, -0.2, 0.7],
        [0.0, 0.000832, 0.0],
        [-1.2, 0.9, 2.4],
    ]
    for rpy in samples:
        matrix = transforms.rpy_to_matrix(rpy)
        quat = transforms.matrix_to_quat_xyzw(matrix)
        assert quat[3] >= 0.0
        assert transforms.quat_xyzw_to_matrix(quat) == pytest.approx(matrix, abs=1e-6)
        assert transforms.rpy_to_matrix(transforms.matrix_to_rpy(matrix)) == pytest.approx(matrix, abs=1e-6)


def test_bundled_model_matches_gold():
    """作品内置的官方 xacro 副本必须独立复现金标，评审不克隆官方仓库也能跑。"""
    chain = run_q2.load_chain(run_q2.BUNDLED_URDF_ROOT)
    actual = run_q2.run_record(RECORD, chain)["objects_link_in_base_footprint"]
    assert actual["xyz"] == pytest.approx(GOLD_OBJECTS["xyz"], abs=1e-4)
    assert actual["rpy"] == pytest.approx(GOLD_OBJECTS["rpy"], abs=1e-5)
    assert actual["quat_xyzw"] == pytest.approx(GOLD_OBJECTS["quat_xyzw"], abs=1e-5)


def test_missing_joint_raises(chain):
    """joint_state 缺关节必须报错，不能静默补零。"""
    partial = dict(ZERO_STATE)
    partial.pop("joint3")
    with pytest.raises(ValueError, match="缺少关节"):
        fk.objects_link_transform(chain, partial)
    with pytest.raises(ValueError, match="未知关节"):
        fk.objects_link_transform(chain, {**ZERO_STATE, "joint9": 0.0})


def test_answer_file(chain):
    """提交的答案逐行可解析、id 与输入对应、数值与重算结果一致。"""
    entry = PROJECT.parents[1]
    records = run_q2.load_records(entry / "inputs" / "q2_demo_input.jsonl")
    lines = (entry / "answers" / "q2_demo_output.jsonl").read_text(encoding="utf-8").splitlines()
    answers = [json.loads(line) for line in lines if line.strip()]
    assert [item["id"] for item in answers] == [record["id"] for record in records]
    for record, answer in zip(records, answers):
        expected = run_q2.run_record(record, chain)
        assert answer == expected


def test_load_records_rejects_non_finite_pose(tmp_path):
    """物体位姿含 nan/inf 必须在第 1 行报错，不能带进求解。"""
    entry = PROJECT.parents[1]
    record = run_q2.load_records(entry / "inputs" / "q2_demo_input.jsonl")[0]
    for bad in (float("nan"), float("inf")):
        broken = json.loads(json.dumps(record))
        broken["scene_object_in_base_footprint"]["xyz"][0] = bad
        path = tmp_path / "bad.jsonl"
        path.write_text(json.dumps(broken) + "\n", encoding="utf-8")
        with pytest.raises(ValueError, match=r"第 1 行.*非有限数值"):
            run_q2.load_records(path)
