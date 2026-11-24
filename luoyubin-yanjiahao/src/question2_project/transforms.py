#!/usr/bin/env python3
"""位姿换算：rpy ↔ 旋转矩阵、旋转矩阵 ↔ 四元数、齐次变换与逆变换。

约定与题面一致：
- rpy 依次为 roll、pitch、yaw，等价于 R = Rz(yaw) · Ry(pitch) · Rx(roll)；
- quat_xyzw 顺序为 [x, y, z, w]；
- 逆变换用转置，不做数值求逆。
"""
import math

import numpy as np


def _rx(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _ry(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rz(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rpy_to_matrix(rpy):
    """R = Rz(yaw) · Ry(pitch) · Rx(roll)。"""
    roll, pitch, yaw = (float(value) for value in rpy)
    return _rz(yaw) @ _ry(pitch) @ _rx(roll)


def matrix_to_rpy(matrix):
    """按 Z-Y-X 顺序从旋转矩阵反解 rpy，处理万向锁附近的奇异点。"""
    pitch = math.atan2(-matrix[2, 0], math.hypot(matrix[0, 0], matrix[1, 0]))
    if abs(math.cos(pitch)) > 1e-9:
        roll = math.atan2(matrix[2, 1], matrix[2, 2])
        yaw = math.atan2(matrix[1, 0], matrix[0, 0])
    else:  # pitch = ±90°，此时 roll 与 yaw 只差一个符号，令 roll = 0。
        roll = 0.0
        yaw = math.atan2(-matrix[0, 1], matrix[1, 1])
    return np.array([roll, pitch, yaw])


def matrix_to_quat_xyzw(matrix):
    """旋转矩阵转四元数，输出 [x, y, z, w]，并取正半球（w ≥ 0）。"""
    trace = matrix[0, 0] + matrix[1, 1] + matrix[2, 2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        w = (matrix[2, 1] - matrix[1, 2]) / scale
        x = 0.25 * scale
        y = (matrix[0, 1] + matrix[1, 0]) / scale
        z = (matrix[0, 2] + matrix[2, 0]) / scale
    elif matrix[1, 1] > matrix[2, 2]:
        scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        w = (matrix[0, 2] - matrix[2, 0]) / scale
        x = (matrix[0, 1] + matrix[1, 0]) / scale
        y = 0.25 * scale
        z = (matrix[1, 2] + matrix[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        w = (matrix[1, 0] - matrix[0, 1]) / scale
        x = (matrix[0, 2] + matrix[2, 0]) / scale
        y = (matrix[1, 2] + matrix[2, 1]) / scale
        z = 0.25 * scale

    quat = np.array([x, y, z, w], dtype=float)
    quat /= np.linalg.norm(quat)
    # 同一旋转有两种符号，统一取正半球，保证输出稳定、可与金标逐位比对。
    if quat[3] < 0.0:
        quat = -quat
    elif quat[3] == 0.0:
        for value in quat[:3]:
            if value != 0.0:
                if value < 0.0:
                    quat = -quat
                break
    return quat


def quat_xyzw_to_matrix(quat):
    """[x, y, z, w] 四元数转旋转矩阵，输入不要求已归一化。"""
    x, y, z, w = (float(value) for value in quat)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise ValueError(f"四元数模长接近 0：{quat!r}")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def make_transform(xyz, rpy):
    """由平移与 rpy 构造 4×4 齐次变换。"""
    transform = np.eye(4)
    transform[:3, :3] = rpy_to_matrix(rpy)
    transform[:3, 3] = [float(value) for value in xyz]
    return transform


def invert_transform(transform):
    """刚体变换求逆：R' = Rᵀ，t' = -Rᵀ t。"""
    inverse = np.eye(4)
    rotation = transform[:3, :3]
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ transform[:3, 3]
    return inverse


def axis_rotation(axis, angle):
    """绕任意单位轴旋转 angle 弧度的 4×4 变换（罗德里格斯公式）。"""
    vector = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(vector)
    if norm < 1e-12:
        raise ValueError(f"旋转轴为零向量：{axis!r}")
    x, y, z = vector / norm
    c, s = math.cos(angle), math.sin(angle)
    transform = np.eye(4)
    transform[:3, :3] = np.array(
        [
            [c + x * x * (1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s],
            [y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s],
            [z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c)],
        ]
    )
    return transform


def prismatic_translation(axis, distance):
    """移动关节：沿单位轴平移 distance 米的 4×4 变换。"""
    vector = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(vector)
    if norm < 1e-12:
        raise ValueError(f"移动轴为零向量：{axis!r}")
    transform = np.eye(4)
    transform[:3, 3] = vector / norm * float(distance)
    return transform


def pose_from_transform(transform):
    """把 4×4 变换拆成 xyz / rpy / quat_xyzw 三个字段。"""
    rotation = transform[:3, :3]
    return {
        "xyz": [float(value) for value in transform[:3, 3]],
        "rpy": [float(value) for value in matrix_to_rpy(rotation)],
        "quat_xyzw": [float(value) for value in matrix_to_quat_xyzw(rotation)],
    }
