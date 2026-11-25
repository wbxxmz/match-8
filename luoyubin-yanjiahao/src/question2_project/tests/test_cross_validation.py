"""用 PyKDL 独立校验题二正向运动学与坐标变换。

与 test_gold.py 的区别：金标只有零位一组数值，查不出“链顺序错但零位碰巧对”；
这里用第二套实现（PyKDL 链式求解器）覆盖零位、逐关节、限位边界和随机姿态。

独立到底：关节链自己从 URDF 回溯、自己构造 KDL 链，不复用 urdf_chain.build_chain，
只把 xacro 展开和产品实现当作被校验对象。

运行：python3 -m pytest src/question2_project/tests
缺少 PyKDL 或官方模型时跳过，并在输出里说明原因。
"""
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))

import fk  # noqa: E402
import run_q2  # noqa: E402
import transforms  # noqa: E402
from urdf_chain import expand_urdf, load_chain  # noqa: E402

kdl = pytest.importorskip("PyKDL", reason="未安装 PyKDL，跳过独立交叉校验")

POSITION_TOLERANCE = 1e-9  # 米；同一份模型下两套实现应达 1e-12 量级
ROTATION_TOLERANCE = 1e-9  # 弧度


def _vector(text, default):
    if text is None or not text.strip():
        return list(default)
    return [float(item) for item in text.split()]


def _parse_urdf(text):
    """自己解析 URDF：关节名、类型、origin、axis、limit。"""
    root = ET.fromstring(text)
    joints = {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        origin = joint.find("origin")
        axis_el = joint.find("axis")
        limit_el = joint.find("limit")
        joints[name] = {
            "type": joint.get("type"),
            "parent": joint.find("parent").get("link"),
            "child": joint.find("child").get("link"),
            "xyz": _vector(origin.get("xyz") if origin is not None else None, (0.0, 0.0, 0.0)),
            "rpy": _vector(origin.get("rpy") if origin is not None else None, (0.0, 0.0, 0.0)),
            "axis": _vector(axis_el.get("xyz") if axis_el is not None else None, (1.0, 0.0, 0.0)),
            "limit": (
                (float(limit_el.get("lower")), float(limit_el.get("upper")))
                if limit_el is not None and limit_el.get("lower") is not None
                else None
            ),
        }
    return joints


def _trace_chain(joints, tip="objects_link", root_link="base_footprint"):
    """独立回溯：从 tip 沿 parent 上溯到 root，不使用产品代码的 build_chain。"""
    by_child = {spec["child"]: name for name, spec in joints.items()}
    names, link = [], tip
    while link != root_link:
        name = by_child.get(link)
        if name is None:
            raise AssertionError(f"从 {tip} 上溯不到 {root_link}，断在 {link}")
        names.append(name)
        link = joints[name]["parent"]
    return list(reversed(names))


def _build_kdl_chain(joints, names):
    """按 KDL 约定构造链：f_tip 用 origin，关节原点与轴向都在父系下表达。"""
    chain, active = kdl.Chain(), []
    for name in names:
        spec = joints[name]
        origin = kdl.Frame(
            kdl.Rotation.RPY(*spec["rpy"]),
            kdl.Vector(*spec["xyz"]),
        )
        axis = kdl.Vector(*spec["axis"])
        axis.Normalize()
        if spec["type"] == "fixed":
            motion = kdl.Joint(name, kdl.Joint.Fixed)
        elif spec["type"] == "prismatic":
            motion = kdl.Joint(name, kdl.Vector(*spec["xyz"]), origin.M * axis, kdl.Joint.TransAxis)
            active.append(name)
        else:
            motion = kdl.Joint(name, kdl.Vector(*spec["xyz"]), origin.M * axis, kdl.Joint.RotAxis)
            active.append(name)
        chain.addSegment(kdl.Segment(spec["child"], motion, origin))
    return chain, active


def _frame_to_matrix(frame):
    matrix = np.eye(4)
    matrix[:3, :3] = [[frame.M[i, j] for j in range(3)] for i in range(3)]
    matrix[:3, 3] = [frame.p[i] for i in range(3)]
    return matrix


@pytest.fixture(scope="module")
def model():
    urdf_root = run_q2.find_urdf_root()
    joints = _parse_urdf(expand_urdf(urdf_root))
    names = _trace_chain(joints)
    chain, active = _build_kdl_chain(joints, names)
    solver = kdl.ChainFkSolverPos_recursive(chain)
    return {
        "joints": joints,
        "names": names,
        "active": active,
        "kdl_chain": chain,  # PyKDL 的求解器持有链引用，必须让它活到测试结束
        "solver": solver,
        "product": load_chain(urdf_root),
    }


def _kdl_fk(model, state):
    positions = kdl.JntArray(len(model["active"]))
    for index, name in enumerate(model["active"]):
        positions[index] = state[name]
    frame = kdl.Frame()
    assert model["solver"].JntToCart(positions, frame) >= 0
    return _frame_to_matrix(frame)


def _compare(actual, expected):
    position_error = float(np.linalg.norm(actual[:3, 3] - expected[:3, 3]))
    # 姿态用旋转矩阵元素差：arccos(trace) 在零角附近病态，会把 1e-16 放大成 1e-8。
    rotation_error = float(np.linalg.norm(actual[:3, :3] - expected[:3, :3]))
    assert position_error <= POSITION_TOLERANCE, f"位置误差 {position_error:.3e} m"
    assert rotation_error <= ROTATION_TOLERANCE, f"姿态矩阵误差 {rotation_error:.3e}"


def _zero():
    return {name: 0.0 for name in fk.JOINTS}


def _random_state(model, rng):
    state = _zero()
    for name in model["active"]:
        limit = model["joints"][name]["limit"]
        if limit is None:
            continue
        state[name] = float(rng.uniform(*limit))
    state["arm_finger_joint2"] = state["arm_finger_joint1"]
    return state


def test_chain_matches_independent_trace(model):
    """产品链与独立回溯出的关节顺序完全一致。"""
    assert [joint.name for joint in model["product"]] == model["names"]


def test_zero_pose(model):
    state = _zero()
    _compare(fk.objects_link_transform(model["product"], state), _kdl_fk(model, state))


def test_each_active_joint(model):
    for name in model["active"]:
        state = _zero()
        state[name] = 0.1 if name == "lift_joint" else 0.4
        _compare(fk.objects_link_transform(model["product"], state), _kdl_fk(model, state))


def test_joint_limit_boundaries(model):
    for name in model["active"]:
        limit = model["joints"][name]["limit"]
        if limit is None:
            continue
        for value in limit:
            state = _zero()
            state[name] = value
            _compare(fk.objects_link_transform(model["product"], state), _kdl_fk(model, state))


def test_100_random_states(model):
    rng = np.random.default_rng(2026)
    max_position = 0.0
    for _ in range(100):
        state = _random_state(model, rng)
        actual = fk.objects_link_transform(model["product"], state)
        expected = _kdl_fk(model, state)
        max_position = max(max_position, float(np.linalg.norm(actual[:3, 3] - expected[:3, 3])))
        _compare(actual, expected)
    print(f"100 组随机姿态最大位置误差 {max_position:.3e} m")


def test_20_object_frame_conversions(model):
    rng = np.random.default_rng(2027)
    for _ in range(20):
        state = _random_state(model, rng)
        xyz = rng.uniform(-2.0, 2.0, 3).tolist()
        rpy = rng.uniform(-3.0, 3.0, 3).tolist()
        objects_in_base = fk.objects_link_transform(model["product"], state)
        actual = transforms.invert_transform(objects_in_base) @ transforms.make_transform(xyz, rpy)
        reference_gripper = _kdl_fk(model, state)
        reference_object = _frame_to_matrix(
            kdl.Frame(kdl.Rotation.RPY(*rpy), kdl.Vector(*xyz))
        )
        expected = np.linalg.inv(reference_gripper) @ reference_object
        _compare(actual, expected)
