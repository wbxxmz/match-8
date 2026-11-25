#!/usr/bin/env python3
"""题三规划：底盘坐标变换、逆运动学、位姿换算（不依赖 Isaac）。

复用题二已经验证过的 URDF 链与 FK：
- `question2_project/urdf_chain.py` 展开官方 xacro 并抽取 base_footprint → objects_link 链；
- `question2_project/fk.py` 做链式正解；
- `question2_project/transforms.py` 做 rpy / 四元数 / 齐次变换。
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

PROJECT_DIR = Path(__file__).resolve().parent
Q2_DIR = PROJECT_DIR.parent / "question2_project"
Q1_LIMITS = PROJECT_DIR.parent / "question1_project" / "config" / "joint_limits.json"
sys.path.insert(0, str(Q2_DIR))

from fk import objects_link_transform  # noqa: E402
from run_q2 import find_urdf_root  # noqa: E402
from transforms import (  # noqa: E402
    axis_rotation,
    invert_transform,
    make_transform,
    pose_from_transform,
    prismatic_translation,
)
from urdf_chain import load_chain  # noqa: E402

# 题面规定的完整 10 关节；FK 缺一个都会拒绝。
JOINTS = [
    "lift_joint", "pitch_joint",
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
    "arm_finger_joint1", "arm_finger_joint2",
]
# objects_link 链上可动的 7 个自由度。
MOVABLE = ["lift_joint", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
DECIMALS = 6

# 求解容差：位置 0.1 mm、姿态 1 mrad，够仿真用且不会无限迭代。
POS_TOL = 1e-4
ORI_TOL = 1e-3
MAX_ITERS = 400
MAX_STEP = 0.2

# 不在 objects_link 链上的关节，IK 内部迭代时用默认值补齐（不影响该链正解）。
CHAIN_FILLER = {"pitch_joint": 0.0, "arm_finger_joint1": 0.3491, "arm_finger_joint2": 0.3491}


def load_limits(path=Q1_LIMITS):
    """读取官方关节限位（与题一共用同一份，保证来源一致）。"""
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    return {name: (float(bound["lower"]), float(bound["upper"])) for name, bound in raw.items()}


def load_robot_chain(urdf_root=None):
    """展开官方模型，返回 base_footprint → objects_link 的关节链。"""
    return load_chain(find_urdf_root(urdf_root))


def full_state(partial):
    """把 7 个可动关节补成题面要求的 10 关节；缺项报错，不静默填零。"""
    missing = [name for name in JOINTS if name not in partial]
    if missing:
        raise ValueError(f"关节状态缺少：{missing}")
    return {name: float(partial[name]) for name in JOINTS}


def fk_objects_link(chain, state):
    """完整 10 关节 → objects_link 在 base_footprint 中的 4×4 位姿。"""
    return objects_link_transform(chain, full_state(state))


def pose_in_parent(transform):
    """4×4 → 题面要求的 xyz / rpy / quat_xyzw，保留 6 位小数。"""
    pose = pose_from_transform(transform)
    return {
        key: [0.0 if round(float(v), DECIMALS) == 0.0 else round(float(v), DECIMALS) for v in values]
        for key, values in pose.items()
    }


def toy_transform_in_base(base_pose, toy_pose):
    """由底盘与玩具的世界位姿，求玩具在底盘系的 4×4 位姿。"""
    base_world = make_transform(base_pose["xyz"], base_pose["rpy"])
    toy_world = make_transform(toy_pose["xyz"], toy_pose["rpy"])
    return invert_transform(base_world) @ toy_world


def toy_pose_in_objects(objects_in_base, toy_in_base):
    """玩具在 objects_link 中的位姿。"""
    return invert_transform(objects_in_base) @ toy_in_base


def top_down_rotation(target_xyz):
    """夹爪朝下的目标姿态：objects_link 的 +Z 指向世界 -Z，+X 指向目标水平方向。"""
    z_axis = np.array([0.0, 0.0, -1.0])
    horizontal = np.array([target_xyz[0], target_xyz[1], 0.0])
    if np.linalg.norm(horizontal) < 1e-9:
        horizontal = np.array([1.0, 0.0, 0.0])
    x_axis = horizontal / np.linalg.norm(horizontal)
    y_axis = np.cross(z_axis, x_axis)
    return np.column_stack([x_axis, y_axis, z_axis])


def approach_rotation(target_xyz, tilt_deg=50.0, heading=None):
    """斜向下抓姿态：在竖直朝下的基础上，把夹爪前方抬起来、朝向目标外侧倾斜。

    heading 给出倾斜平面内的水平方向（默认“底座→目标”）；长条玩具传长轴方向，
    让两指沿短轴合拢，避免手指顺着身体长度插到端部。
    """
    vertical = top_down_rotation(target_xyz)
    if heading is None:
        horizontal = np.array([target_xyz[0], target_xyz[1], 0.0])
    else:
        horizontal = np.array([heading[0], heading[1], 0.0])
    if np.linalg.norm(horizontal) < 1e-9:
        horizontal = np.array([1.0, 0.0, 0.0])
    horizontal /= np.linalg.norm(horizontal)
    axis = np.cross([0.0, 0.0, 1.0], horizontal)  # 水平且垂直于倾斜平面内的方向
    axis /= np.linalg.norm(axis)
    angle = math.radians(tilt_deg)
    c, s = math.cos(angle), math.sin(angle)
    skew = np.array([[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]])
    return (np.eye(3) - s * skew + (1.0 - c) * (skew @ skew)) @ vertical


def rotation_error(current, target):
    """旋转矩阵误差 → 旋转向量（小角度近似），用于 IK 的 6 维误差。"""
    error = target @ current.T
    return 0.5 * np.array([
        error[2, 1] - error[1, 2],
        error[0, 2] - error[2, 0],
        error[1, 0] - error[0, 1],
    ])


def solve_ik(chain, target_xyz, target_rot, seed, limits, iters=MAX_ITERS):
    """阻尼最小二乘 IK：位置 + 姿态同时收敛，返回 7 个可动关节与残差。

    数值雅可比每列一次 FK，7 列 × 最多 400 次迭代，离线毫秒级。
    """
    state = {name: float(seed.get(name, 0.0)) for name in MOVABLE}
    target_xyz = np.asarray(target_xyz, dtype=float)
    damping = 0.02
    for _ in range(iters):
        transform = objects_link_transform(chain, {**CHAIN_FILLER, **state})
        pos_error = target_xyz - transform[:3, 3]
        ori_error = rotation_error(transform[:3, :3], target_rot)
        if np.linalg.norm(pos_error) < POS_TOL and np.linalg.norm(ori_error) < ORI_TOL:
            break
        jacobian = np.zeros((6, len(MOVABLE)))
        for index, name in enumerate(MOVABLE):
            probe = {**CHAIN_FILLER, **state}
            probe[name] += 1e-5
            probe_transform = objects_link_transform(chain, probe)
            jacobian[:3, index] = (probe_transform[:3, 3] - transform[:3, 3]) / 1e-5
            jacobian[3:, index] = rotation_error(probe_transform[:3, :3], transform[:3, :3]) / 1e-5
        # 姿态误差放权 0.3，避免为了转手腕牺牲位置精度。
        error = np.concatenate([pos_error, 0.3 * ori_error])
        jacobian[3:] *= 0.3
        delta = jacobian.T @ np.linalg.solve(
            jacobian @ jacobian.T + (damping ** 2) * np.eye(6), error
        )
        largest = float(np.max(np.abs(delta)))
        scale = 1.0 if largest <= MAX_STEP else MAX_STEP / largest
        for index, name in enumerate(MOVABLE):
            lower, upper = limits[name]
            state[name] = float(np.clip(state[name] + scale * delta[index], lower, upper))
    transform = objects_link_transform(chain, {**CHAIN_FILLER, **state})
    return (
        state,
        float(np.linalg.norm(target_xyz - transform[:3, 3])),
        float(np.linalg.norm(rotation_error(transform[:3, :3], target_rot))),
    )


# 自然伸手姿态（离线 FK 扫出来的标称解）：底座朝向目标、肘部适度弯、手腕不反折。
# 官方输入的起点是收拢停车位（joint2≈2.79、joint5≈-2.27），从它出发解出来的姿态
# 会让底座转 140° 反向够、手腕反折 130°；用这个标称姿态当种子则得到面向目标的舒展解。
REACH_POSTURE = {
    "joint2": 0.15, "joint3": 1.15, "joint4": 0.0, "joint5": -1.2, "joint6": 0.0,
}


def reach_seed(azimuth):
    """按目标方位角构造伸手初值：joint1 直接指向目标，其余取标称姿态。"""
    seed = dict(REACH_POSTURE)
    seed["joint1"] = float(np.clip(azimuth, -3.12, 3.12))
    return seed


def solve_best(chain, target_xyz, target_rot, seeds, limits, posture=None):
    """多初值求解：残差合格者里取最接近标称姿态的那个。

    只看残差会在多个等价解里随便挑（反向转身、手腕反折），姿态会很怪；
    因此先过滤“位置 ≤1 毫米、姿态 ≤5 毫弧度”，再比与标称姿态的关节偏差平方和。
    """
    candidates = []
    for seed in seeds:
        state, pos_error, ori_error = solve_ik(chain, target_xyz, target_rot, seed, limits)
        good = pos_error <= 1e-3 and ori_error <= 5e-3
        if good and posture:
            cost = float(sum((state[k] - v) ** 2 for k, v in posture.items() if k in state))
        else:
            cost = float("inf")
        candidates.append((0 if good else 1, cost, pos_error, state, ori_error))
    _, _, pos_error, state, ori_error = min(candidates, key=lambda item: item[:3])
    return state, pos_error, ori_error



def interpolate(start, end, ratio):
    """关节空间线性插值；ratio 已经由调用方做过平滑。"""
    return {name: start[name] + (end[name] - start[name]) * ratio for name in MOVABLE}


def smoothstep(ratio):
    """两端速度为 0 的缓动，避免起步与停止时关节速度突变。"""
    ratio = min(max(ratio, 0.0), 1.0)
    return ratio * ratio * (3.0 - 2.0 * ratio)


def link_origins_below_floor(chain, state, floor_z=0.0):
    """沿链累乘，返回低于地面的连杆原点；只用于运行期安全检查。"""
    transform = np.eye(4)
    below = []
    for joint in chain:
        transform = transform @ make_transform(joint.xyz, joint.rpy)
        if joint.movable and joint.name in state:
            value = float(state[joint.name])
            transform = transform @ (
                prismatic_translation(joint.axis, value)
                if joint.type == "prismatic"
                else axis_rotation(joint.axis, value)
            )
        if transform[2, 3] < floor_z:
            below.append((joint.name, round(float(transform[2, 3]), 4)))
    return below
