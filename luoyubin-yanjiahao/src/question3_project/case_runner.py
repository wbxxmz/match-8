#!/usr/bin/env python3
"""题三单例执行：置初值、规划、下降触碰、合拢、取证并组装答案。"""
import math

import numpy as np

import config
import planner
from contact import decide_touch
from planning import choose_grasp


def _toy_size(stage, toy_path):
    """玩具包围盒尺寸（世界系）。objects_link 是掌部参考点，不能用固定 3 cm 衡量碰没碰到。"""
    from pxr import Usd, UsdGeom

    bounds = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(), [UsdGeom.Tokens.default_]
    ).ComputeWorldBound(stage.GetPrimAtPath(toy_path)).ComputeAlignedRange()
    return np.asarray(bounds.GetSize(), dtype=float)


def _yaw_gap_deg(now, reference):
    gap = math.degrees(
        math.atan2(now[1, 0], now[0, 0]) - math.atan2(reference[1, 0], reference[0, 0])
    )
    return (gap + 180.0) % 360.0 - 180.0


def run_case(arm, record, index, finger_gap=0.0):
    """跑一个测例并返回答案字段；过渡动作只在 index>0 时执行。"""
    sim = arm.sim
    chain = arm.chain
    limits = arm.limits
    base_pose = arm.base_pose
    name = record["name"]
    case_id = record["id"]
    print(f"\n=== {case_id} ({name}) ===", flush=True)

    initial = planner.full_state(record["joint_state"])
    if index:
        sim.contact_monitor.pause()  # 过渡动作不计入本例接触证据
        print(
            f"过渡：松指后抬到 {config.TRANSFER_HEIGHT:.2f} 米，再平滑插值到本例初值",
            flush=True,
        )
        arm.retract_and_transfer(record["joint_state"])
    sim.teleport(initial)
    sim.place_toy_at_input(name, record["scene_object_in_world"])
    sim.contact_monitor.begin(config.TOY_PRIMS[name])
    actual = sim.read_state()
    initial_error = max(abs(actual[key] - initial[key]) for key in planner.JOINTS)
    print(f"初值读回最大误差 {initial_error:.6f} rad/m", flush=True)

    base_from_input = planner.toy_transform_in_base(base_pose, record["scene_object_in_world"])
    toy_now = sim.toy_pose(name)
    base_from_scene = planner.toy_transform_in_base(base_pose, toy_now)
    delta = np.linalg.norm(base_from_input[:3, 3] - base_from_scene[:3, 3])
    print(
        f"玩具在底盘系：输入 {np.round(base_from_input[:3, 3], 4).tolist()}，"
        f"场景 {np.round(base_from_scene[:3, 3], 4).tolist()}，差 {delta:.4f} m",
        flush=True,
    )
    # 玩具包围盒：同时给出玩具自身尺寸，便于解释最终距离的几何下限。
    toy_size = _toy_size(sim.stage, config.TOY_PRIMS[name])
    toy_radius = float(np.linalg.norm(toy_size) / 2)
    print(
        f"玩具包围盒尺寸 {np.round(toy_size, 4).tolist()} m，"
        f"半对角线 {toy_radius:.4f} m",
        flush=True,
    )

    target_xyz = base_from_scene[:3, 3].copy()
    pre_xyz, grasp_xyz, grasp_rotation, open_grasp, mode_name = choose_grasp(
        name, target_xyz, toy_size
    )
    pregrasp, pre_error, _ = arm.solve_target(pre_xyz, initial, grasp_rotation)
    grasp, grasp_error, grasp_ori = arm.solve_target(grasp_xyz, initial, grasp_rotation)
    if open_grasp:
        for pose in (pregrasp, grasp):
            pose["arm_finger_joint1"] = config.FINGER_OPEN
            pose["arm_finger_joint2"] = config.FINGER_OPEN
    print(
        f"IK 预抓取位置残差 {pre_error:.5f} m，抓取位置残差 {grasp_error:.5f} m，"
        f"姿态残差 {grasp_ori:.5f} rad，模式 {mode_name}",
        flush=True,
    )

    # 抬升与转向合成一段：目标取在玩具方位、半径 0.40 米、高 0.30 米。
    # 分两段（先举高再平移）看着像无意义升高；合成一段是连续的斜向上伸手。
    # 高度 0.30 米仍高于所有玩具（最高约 0.2 米），旋转时不会扫地。
    azimuth = math.atan2(float(target_xyz[1]), float(target_xyz[0]))
    transit_xyz = np.array([0.40 * math.cos(azimuth), 0.40 * math.sin(azimuth), 0.30])
    transit, transit_error, _ = arm.solve_target(
        transit_xyz, initial, planner.top_down_rotation(transit_xyz)
    )
    if open_grasp:
        transit["arm_finger_joint1"] = config.FINGER_OPEN
        transit["arm_finger_joint2"] = config.FINGER_OPEN
    print(f"  转运姿态（高 0.30 米）解残差 {transit_error:.4f} m", flush=True)
    for label, target, frames in (
        ("抬升并转向玩具", transit, 100),
        ("下降到预抓取", pregrasp, 90),
    ):
        print(f"-- {label}", flush=True)
        arm.move_towards(target, frames, name)
        below = planner.link_origins_below_floor(chain, sim.read_state())
        if below:
            print("  警告：连杆原点低于地面", below, flush=True)
    print("-- 等待预抓取到位", flush=True)
    settle_error = arm.settle_to(pregrasp, pre_xyz)
    print(f"  预抓取到位误差 {settle_error:.4f} m", flush=True)

    print("-- 下降到玩具", flush=True)
    toy_before = planner.toy_transform_in_base(base_pose, sim.toy_pose(name))[:3, 3].copy()
    if open_grasp:
        # 宽抓/侧抱：不再“碰到就停”，而是降到两指包住玩具的高度，再合拢夹住。
        arm.move_towards(grasp, 120, name)
        print("-- 等待宽抓到位", flush=True)
        wide_error = arm.settle_to(grasp, grasp_xyz)
        print(f"  宽抓到位误差 {wide_error:.4f} m", flush=True)
        sim.advance(20)
        best_distance = arm.distance(name, arm.objects_pose())
        rebound = False
        descent_effort = arm.finger_effort()
    else:
        best_distance, rebound, descent_effort = arm.descend_until_contact(
            grasp, target_xyz, name
        )
    # 接触后先让物理稳定，再读最终关节；否则拿到的可能是插值过程中的瞬时状态。
    sim.advance(20)
    before_close = arm.distance(name, arm.objects_pose())
    if rebound and not open_grasp:
        # 已经停在玩具表面：再合拢只会把玩具推走，跳过。
        # 但下降途中压到玩具时测到的指力矩要保留，否则报告会变成“接触但几乎无力”。
        print("-- 已停在玩具表面，跳过合拢", flush=True)
        peak_effort = descent_effort
    else:
        print("-- 合拢手指", flush=True)
        peak_effort = arm.close_fingers(steps=90 if open_grasp else 60, gap=finger_gap)
    current_distance = arm.distance(name, arm.objects_pose())
    effort = max(peak_effort, arm.finger_effort())
    toy_shift = float(np.linalg.norm(
        planner.toy_transform_in_base(base_pose, sim.toy_pose(name))[:3, 3] - toy_before
    ))
    # 触碰的物理证据：PhysX 上报的真实碰撞对，或指关节被夹住、下降触底停顿、玩具被推离原位。
    # 指力矩测不到“自上而下压”，因为力沿指连杆方向，不绕指关节轴。
    sensor_event = sim.contact_monitor.target
    contact = effort > config.EFFORT_CONTACT or rebound or toy_shift > config.TOY_DISPLACEMENT
    if sensor_event is not None:
        print(
            f"  PhysX 接触对 {sensor_event['actors']} "
            f"分离 {sensor_event['separation_m']:.6f} m，"
            f"冲量 {sensor_event['impulse_ns']:.6f} N·s",
            flush=True,
        )
    else:
        print(f"  PhysX 接触 未上报（本测例共 {sim.contact_monitor.events} 个有效接触点）", flush=True)
    if sim.contact_monitor.unexpected:
        print(f"  意外接触 {sim.contact_monitor.unexpected[0]['actors']}", flush=True)
    print(
        f"接触距离 {current_distance:.4f} m（合拢前 {before_close:.4f} m，最优 {best_distance:.4f} m，"
        f"触底回升 {'是' if rebound else '否'}，峰值指力矩 {effort:.3f} N·m，"
        f"玩具位移 {toy_shift * 1000:.1f} mm，接触 {'有' if contact else '无'}）",
        flush=True,
    )
    efforts = np.asarray(sim.articulation.get_measured_joint_efforts(), dtype=float).reshape(-1)
    finger_efforts = {
        joint: round(float(efforts[index]), 4)
        for index, joint in enumerate(sim.dof_names)
        if "finger" in joint
    }
    print("  指关节力矩", finger_efforts, flush=True)

    final_state = sim.read_state()
    final_errors = {
        key: (final_state[key], limits[key])
        for key in planner.JOINTS
        if key in limits and not (limits[key][0] - 1e-4 <= final_state[key] <= limits[key][1] + 1e-4)
    }
    if final_errors:
        raise RuntimeError(f"{case_id} 最终关节越限：{final_errors}")

    objects_in_base = planner.fk_objects_link(chain, final_state)
    objects_pose = planner.pose_in_parent(objects_in_base)
    usd_objects_in_base = planner.toy_transform_in_base(base_pose, sim.world_pose(config.OBJECTS_PRIM))
    usd_delta = float(
        np.linalg.norm(usd_objects_in_base[:3, 3] - np.asarray(objects_pose["xyz"]))
    )
    # 自检：底盘有没有被推走，用来解释 USD/FK 偏差。
    base_now = sim.world_pose(config.BASE_PRIM)
    base_shift = float(np.linalg.norm(np.asarray(base_now["xyz"]) - np.asarray(base_pose["xyz"])))
    print(f"  底盘位移 {base_shift:.4f} m", flush=True)
    # 题面 2.4 节规定 scene_object_in_world 就是玩具的世界位姿，所以答案主字段按输入算（恐龙 rpy=0）。
    # 场景里恐龙刚体自带 rotateXYZ=(0,0,-90)，仿真实际位姿与输入口径差约 90°，单独存为附加字段。
    toy_in_base = planner.toy_transform_in_base(base_pose, sim.toy_pose(name))
    toy_in_base_from_input = planner.toy_transform_in_base(
        record["base_footprint_in_world"], record["scene_object_in_world"]
    )
    toy_in_objects = planner.toy_pose_in_objects(objects_in_base, toy_in_base_from_input)
    toy_in_objects_sim = planner.toy_pose_in_objects(objects_in_base, toy_in_base)
    yaw_gap = _yaw_gap_deg(toy_in_base, toy_in_base_from_input)
    input_gap = float(np.linalg.norm(toy_in_base[:3, 3] - toy_in_base_from_input[:3, 3]))
    print(
        f"  玩具位姿：主字段按题面输入（rpy=0）计算，仿真实测另存；"
        f"两者相差 {input_gap:.4f} m / {yaw_gap:+.1f}°（资产自带旋转 + 触碰推动所致）",
        flush=True,
    )
    touch_distance = float(np.linalg.norm(toy_in_base[:3, 3] - objects_in_base[:3, 3]))
    # 触碰判据：PhysX 接触报告或物理证据（回升/力矩/位移）优先；距离只作几何兜底——
    # 玩具被撞飞时同样会上报接触，但它已不在夹爪工作范围，不能算“碰到”。
    distance_limit = max(config.TOUCH_DISTANCE, toy_radius)
    passed = decide_touch(contact, sensor_event) and touch_distance <= distance_limit
    print(
        f"完成：最终距离 {touch_distance:.4f} m（几何参考上限 {distance_limit:.4f} m），"
        f"指力矩 {effort:.3f} N·m，USD/FK 偏差 {usd_delta:.5f} m，{'PASS' if passed else 'FAIL'}",
        flush=True,
    )

    return {
        "id": case_id,
        "final_joint_state": {key: round(float(final_state[key]), 6) for key in planner.JOINTS},
        "objects_link_in_base_footprint": objects_pose,
        "scene_object_in_objects_link": planner.pose_in_parent(toy_in_objects),
        "scene_object_in_objects_link_sim": planner.pose_in_parent(toy_in_objects_sim),
        "scene_object_input_gap_m": round(input_gap, 6),
        "scene_object_input_gap_yaw_deg": round(yaw_gap, 3),
        "touch_distance_m": round(touch_distance, 6),
        "min_distance_m": round(min(best_distance, touch_distance), 6),
        "contact_detected": bool(passed),
        "physx_contact": sensor_event,
        "unexpected_contact": sim.contact_monitor.unexpected[0] if sim.contact_monitor.unexpected else None,
        "finger_effort_nm": round(effort, 4),
        "toy_displacement_m": round(toy_shift, 6),
        "initial_state_error_max": round(initial_error, 6),
    }
