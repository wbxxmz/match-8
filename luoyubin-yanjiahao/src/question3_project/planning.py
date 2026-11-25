#!/usr/bin/env python3
"""题三规划：目标点选择、多初值 IK 与不启动 Isaac 的干跑预检。"""
import math

import numpy as np

import planner
from config import (
    FRONT_GRASP_FACE_HEIGHT,
    FRONT_GRASP_PULLBACK,
    FRONT_GRASP_TOYS,
    GRASP_YAW_DEG,
    PREGRASP_HEIGHT,
    SIDE_WIDE_CLEARANCE,
    SIDE_WIDE_TOYS,
    WIDE_GRASP_AXIS,
    WIDE_GRASP_CLEARANCE,
    WIDE_GRASP_TOYS,
)


def plan_for_target(chain, limits, target_xyz, rotation=None, seed_state=None):
    """给定底盘系目标点求关键关节；实跑与 --dry-run 共用同一套规划。

    主种子按目标方位角摆放底座，避免解出“反向转身 + 手腕反折”的怪异姿态；
    底盘已锁定，反作用不会再把这台轮式机器人推走。
    """
    rotation = planner.top_down_rotation(target_xyz) if rotation is None else rotation
    azimuth = math.atan2(float(target_xyz[1]), float(target_xyz[0]))
    seeds = [planner.reach_seed(azimuth + offset) for offset in (0.0, -0.35, 0.35, -0.7, 0.7)]
    posture = {"joint1": azimuth, **planner.REACH_POSTURE}
    if seed_state is not None:
        seeds.append(seed_state)
    return planner.solve_best(chain, target_xyz, rotation, seeds, limits, posture=posture)


def choose_grasp(name, target_xyz, toy_size):
    """按玩具类型选抓法，返回（预抓取点, 抓取点, 抓取姿态, 是否张开两指, 模式名）。"""
    pre_xyz = target_xyz + np.array([0.0, 0.0, PREGRASP_HEIGHT])
    wide_grasp = name in WIDE_GRASP_TOYS
    side_grasp = name in SIDE_WIDE_TOYS
    front_grasp = name in FRONT_GRASP_TOYS
    yaw = GRASP_YAW_DEG.get(name, 0.0)
    if wide_grasp:
        # 方块：竖直朝下，两指沿底盘 x 轴张开，贴合方块的两个侧面，
        # 掌部停在中心上方 5 毫米，两指从两侧包住后再合拢。
        grasp_xyz = target_xyz + np.array([0.0, 0.0, WIDE_GRASP_CLEARANCE[name]])
        grasp_rotation = planner.top_down_side_rotation(np.array(WIDE_GRASP_AXIS[name]))
    elif front_grasp:
        # 软玩具：掌心朝脸部（底盘 +x），两指沿水平 y 张开，
        # 指尖落在朝向机器人的那个面上，夹住脸部左右两侧而不是头两侧。
        approach = np.array([target_xyz[0], target_xyz[1], 0.0])
        approach /= np.linalg.norm(approach)
        grasp_xyz = target_xyz - approach * (toy_size[0] / 2.0)
        grasp_xyz[2] += FRONT_GRASP_FACE_HEIGHT
        grasp_rotation = planner.front_grasp_rotation(target_xyz)
        pre_xyz = grasp_xyz - approach * FRONT_GRASP_PULLBACK + np.array([0.0, 0.0, 0.01])
    elif side_grasp:
        # 长条玩具：竖直朝下，两指沿短轴张开，掌部停在中心上方 2 厘米，
        # 从身体两侧包住上半身再合拢，不往下压。
        short_axis = int(np.argmin(toy_size[:2]))
        axis = np.zeros(3)
        axis[short_axis] = 1.0
        grasp_xyz = target_xyz + np.array([0.0, 0.0, SIDE_WIDE_CLEARANCE])
        grasp_rotation = planner.yaw_rotation(
            planner.top_down_side_rotation(axis), yaw
        )
    else:
        grasp_xyz = target_xyz
        # 倾斜接近：手指从斜上方包住玩具，比竖直朝下多留几厘米手腕余量。
        grasp_rotation = planner.yaw_rotation(planner.approach_rotation(target_xyz), yaw)
    open_grasp = wide_grasp or side_grasp or front_grasp
    mode_name = "宽抓" if wide_grasp else ("侧抱" if side_grasp else ("正面抓" if front_grasp else "斜抓"))
    return pre_xyz, grasp_xyz, grasp_rotation, open_grasp, mode_name


def dry_run(records, chain, limits):
    """不启动 Isaac，只用输入里的位姿做规划预检。

    输入自带底盘与玩具的世界位姿，因此无需加载 USD 就能验证：
    目标点换算、IK 是否收敛、解出的姿态是否穿地。
    """
    print("\nDRY-RUN：只做规划，不启动仿真、不写答案", flush=True)
    failed = 0
    for record in records:
        name = record["name"]
        base_pose = {"xyz": record["base_footprint_in_world"]["xyz"], "rpy": record["base_footprint_in_world"]["rpy"]}
        target = planner.toy_transform_in_base(base_pose, record["scene_object_in_world"])[:3, 3]
        wide = name in WIDE_GRASP_TOYS
        side = name in SIDE_WIDE_TOYS
        front = name in FRONT_GRASP_TOYS
        yaw = GRASP_YAW_DEG.get(name, 0.0)
        if wide:
            rotation = planner.top_down_side_rotation(np.array([1.0, 0.0, 0.0]))
            points = (
                ("预抓取", target + np.array([0.0, 0.0, PREGRASP_HEIGHT])),
                ("抓取", target + np.array([0.0, 0.0, WIDE_GRASP_CLEARANCE[name]])),
            )
        elif front:
            # 干跑拿不到玩具包围盒，软玩具的半长按本场景的 3.7 厘米近似
            approach = np.array([target[0], target[1], 0.0])
            approach /= np.linalg.norm(approach)
            face = target - approach * 0.037
            face[2] += FRONT_GRASP_FACE_HEIGHT
            rotation = planner.front_grasp_rotation(target)
            points = (
                ("预抓取", face - approach * FRONT_GRASP_PULLBACK + np.array([0.0, 0.0, 0.01])),
                ("抓取", face),
            )
        elif side:
            # 干跑拿不到玩具包围盒，按本场景的短轴（底盘 x）近似
            rotation = planner.yaw_rotation(
                planner.top_down_side_rotation(np.array([1.0, 0.0, 0.0])), yaw
            )
            points = (
                ("预抓取", target + np.array([0.0, 0.0, PREGRASP_HEIGHT])),
                ("抓取", target + np.array([0.0, 0.0, SIDE_WIDE_CLEARANCE])),
            )
        else:
            rotation = planner.yaw_rotation(planner.approach_rotation(target), yaw)
            points = (
                ("预抓取", target + np.array([0.0, 0.0, PREGRASP_HEIGHT])),
                ("抓取", target),
            )
        print(f"  {record['id']}({name})：玩具在底盘系 {np.round(target, 4).tolist()}", flush=True)
        for label, point in points:
            state, pos_error, ori_error = plan_for_target(chain, limits, point, rotation=rotation)
            # 头部与手指不参与链求解，用输入初值补全后再做穿地检查。
            full = planner.full_state({**record["joint_state"], **state})
            below = planner.link_origins_below_floor(chain, full)
            ok = pos_error <= 1e-3 and ori_error <= 5e-3 and not below
            failed += 0 if ok else 1
            print(
                f"    {label} {np.round(point, 4).tolist()}：位置残差 {pos_error:.6f} m，"
                f"姿态残差 {ori_error:.6f} rad，穿地 {'是' if below else '否'}"
                f"{'' if not below else below} -> {'可达' if ok else '不可达'}",
                flush=True,
            )
        print(
            "    关节近似解 joint1~6 = "
            + ", ".join(f"{state[key]:.4f}" for key in ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")),
            flush=True,
        )
    print(f"\nDRY-RUN 完成：{len(records)} 个测例 × 2 个目标点，{failed} 个不可达", flush=True)
    return 1 if failed else 0
