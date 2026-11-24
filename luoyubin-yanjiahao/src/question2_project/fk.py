#!/usr/bin/env python3
"""题二正向运动学：完整 joint_state → objects_link 在 base_footprint 中的 4×4 位姿。

变换顺序：对链上每个关节，先按 URDF origin 做平移 + rpy，再叠加关节运动；
固定关节只做 origin，转动关节绕 URDF 给定的 axis 旋转，移动关节沿 axis 平移。
"""
import math

import numpy as np

from transforms import axis_rotation, make_transform, prismatic_translation

# 题面规定的完整关节状态，缺一个都视为非法输入。
JOINTS = [
    "lift_joint", "pitch_joint",
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
    "arm_finger_joint1", "arm_finger_joint2",
]


def check_joint_state(joint_state):
    """校验输入正好包含 10 个关节且数值有限，返回 float 化后的副本。"""
    if not isinstance(joint_state, dict):
        raise ValueError(f"joint_state 应为对象，实际是 {type(joint_state).__name__}")
    missing = [name for name in JOINTS if name not in joint_state]
    if missing:
        raise ValueError(f"joint_state 缺少关节：{missing}")
    unknown = sorted(set(joint_state) - set(JOINTS))
    if unknown:
        raise ValueError(f"joint_state 含未知关节：{unknown}")
    state = {}
    for name in JOINTS:
        value = joint_state[name]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{name} 不是数值：{value!r}")
        value = float(value)
        if not math.isfinite(value):
            raise ValueError(f"{name} 不是有限数值：{value!r}")
        state[name] = value
    return state


def objects_link_transform(chain, joint_state):
    """沿关节链累乘，返回 base_footprint → objects_link 的 4×4 变换。"""
    state = check_joint_state(joint_state)
    transform = np.eye(4)
    for joint in chain:
        transform = transform @ make_transform(joint.xyz, joint.rpy)
        if not joint.movable:
            continue
        if joint.name not in state:
            raise ValueError(f"链上关节 {joint.name!r} 不在 joint_state 中")
        if joint.type == "prismatic":
            transform = transform @ prismatic_translation(joint.axis, state[joint.name])
        else:  # revolute / continuous
            transform = transform @ axis_rotation(joint.axis, state[joint.name])
    return transform
