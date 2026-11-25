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


def grasp_frame(approach, finger_axis):
    """由接近方向与手指张开方向构造抓取姿态。

    `objects_link` 的 +X 是接近方向（从手腕指向指尖），+Y 是两指张开方向，
    +Z 是手指转轴。finger_axis 会投影到垂直于 approach 的平面上。
    """
    x_axis = np.asarray(approach, dtype=float)
    x_axis = x_axis / np.linalg.norm(x_axis)
    y_axis = np.asarray(finger_axis, dtype=float)
    y_axis = y_axis - np.dot(y_axis, x_axis) * x_axis
    if np.linalg.norm(y_axis) < 1e-9:
        y_axis = np.array([0.0, 1.0, 0.0]) - x_axis[1] * x_axis
    y_axis = y_axis / np.linalg.norm(y_axis)
    z_axis = np.cross(x_axis, y_axis)
    return np.column_stack([x_axis, y_axis, z_axis])


def front_grasp_rotation(target_xyz, tilt_deg=40.0):
    """正面抓取：掌心朝玩具正面（水平指向玩具），略向下倾，两指沿水平横向张开。

    脸部朝侧面时，两指沿 y 张开、从左右两侧夹住正脸；指尖要落到玩具远侧，
    让手指长度覆盖整个头部。纯水平时手腕关节几乎贴地（joint5 原点只剩 8 毫米），
    下倾 40° 让手腕抬到 15 厘米，掌心仍以 x 分量为主。
    """
    horizontal = np.array([target_xyz[0], target_xyz[1], 0.0])
    if np.linalg.norm(horizontal) < 1e-9:
        horizontal = np.array([1.0, 0.0, 0.0])
    horizontal /= np.linalg.norm(horizontal)
    tilt = math.radians(tilt_deg)
    approach = np.array([
        horizontal[0] * math.cos(tilt),
        horizontal[1] * math.cos(tilt),
        -math.sin(tilt),
    ])
    return grasp_frame(approach, [0.0, 1.0, 0.0])


def top_down_rotation(target_xyz):
    """竖直朝下：+X 指向世界 -Z，两指沿目标水平径向张开。"""
    horizontal = np.array([target_xyz[0], target_xyz[1], 0.0])
    if np.linalg.norm(horizontal) < 1e-9:
        horizontal = np.array([1.0, 0.0, 0.0])
    return grasp_frame([0.0, 0.0, -1.0], horizontal)


def top_down_side_rotation(short_axis):
    """竖直朝下、两指沿指定水平短轴张开的姿态，用于长条玩具侧抱。

    长条玩具若仍按径向张指，两指会卡在身体两端；这里把 +Y 转到短轴方向，
    让两指从身体两侧包住上半身。
    """
    return grasp_frame([0.0, 0.0, -1.0], np.array([short_axis[0], short_axis[1], 0.0]))


def yaw_rotation(rotation, angle_deg):
    """绕竖直轴旋转抓取姿态，只改变两指张开的方位，不改变接近方向。"""
    angle = math.radians(angle_deg)
    c, s = math.cos(angle), math.sin(angle)
    yaw = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return yaw @ rotation


def approach_rotation(target_xyz, tilt_deg=50.0):
    """斜向下抓：接近方向从竖直向下朝目标外侧倾斜，两指沿切向张开。

    手腕留在目标外侧，手指从斜上方包住玩具，比纯竖直多留一点手腕余量。
    """
    horizontal = np.array([target_xyz[0], target_xyz[1], 0.0])
    if np.linalg.norm(horizontal) < 1e-9:
        horizontal = np.array([1.0, 0.0, 0.0])
    horizontal /= np.linalg.norm(horizontal)
    angle = math.radians(tilt_deg)
    approach = math.cos(angle) * np.array([0.0, 0.0, -1.0]) + math.sin(angle) * horizontal
    finger_axis = np.cross([0.0, 0.0, 1.0], horizontal)
    return grasp_frame(approach, finger_axis)


def rotation_error(current, target):
    """旋转矩阵误差 → 旋转向量（轴角），对 180° 也有效。

    只取反对称部分（`error[2,1]-error[1,2]` 等）在角度接近 180° 时会归零，
    让 IK 误以为已经到位，实际手腕是反的。这里用轴角公式：一般情况按
    sin(angle) 缩放，接近 180° 时改从对角元恢复转轴。
    """
    error = target @ current.T
    cos_angle = float(np.clip((np.trace(error) - 1.0) / 2.0, -1.0, 1.0))
    angle = math.acos(cos_angle)
    if angle < 1e-9:
        return np.zeros(3)
    if angle < math.pi - 1e-4:
        scale = angle / (2.0 * math.sin(angle))
        return scale * np.array([
            error[2, 1] - error[1, 2],
            error[0, 2] - error[2, 0],
            error[1, 0] - error[0, 1],
        ])
    # 180°：R = 2 a aᵀ - I，从最大对角元恢复转轴
    index = int(np.argmax(np.diag(error)))
    axis = np.zeros(3)
    axis[index] = math.sqrt(max((error[index, index] + 1.0) / 2.0, 0.0))
    if axis[index] < 1e-8:
        return np.array([angle, 0.0, 0.0])
    for other in range(3):
        if other != index:
            axis[other] = error[other, index] / (2.0 * axis[index])
    norm = float(np.linalg.norm(axis))
    if norm < 1e-9:
        return np.zeros(3)
    return angle * axis / norm


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
            # 与位置列同号约定：J = -d(误差)/dq，解 J·δ = 误差。
            jacobian[3:, index] = 0.3 * rotation_error(
                transform[:3, :3], probe_transform[:3, :3]
            ) / 1e-5
        # 姿态误差放权 0.3，避免为了转手腕牺牲位置精度。
        error = np.concatenate([pos_error, 0.3 * ori_error])
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
        below = link_origins_below_floor(chain, {**CHAIN_FILLER, **state})
        if good and posture:
            cost = float(sum((state[k] - v) ** 2 for k, v in posture.items() if k in state))
        else:
            cost = float("inf")
        # 先保证不穿地，再比与标称姿态的偏差。
        rank = 0 if (good and not below) else 1
        candidates.append((rank, cost, pos_error, state, ori_error))
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
