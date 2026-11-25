#!/usr/bin/env python3
"""题三答案离线自检：不需要 Isaac Sim，只读答案、输入、日志与视频。

检查项：
1. 三行答案的 id 与输入一一对应，10 关节齐全且在限位内；
2. 用最终关节重算 FK，与答案里的 objects_link 位姿一致；
3. 答案里的玩具位姿能正变换回输入给定的世界位姿（逆变换闭环）；按输入位姿算的那一份必须严格闭环；
4. 三例 contact_detected 为真，距离不超过 max(3 厘米, 玩具包络半径)；
5. 日志含三次 PASS、无 error/warning；视频存在且非空。

用法（作品根目录）：
    python3 src/question3_project/verify_q3.py
"""
import json
import os
import platform
import re
import sys
from pathlib import Path

import numpy as np

PROJECT_DIR = Path(__file__).resolve().parent
ENTRY_ROOT = PROJECT_DIR.parents[1]
sys.path.insert(0, str(PROJECT_DIR))

import planner  # noqa: E402

DISTANCE_FLOOR = 0.03      # 距离判据下限（米），与 touch_toy.py 一致
MIN_VIDEO_BYTES = 1_000_000


def fail(message):
    print(f"FAIL  {message}")
    return 1


def load_jsonl(path):
    return [json.loads(line) for line in Path(path).read_text(encoding="utf-8").splitlines() if line.strip()]


def pose_matrix(pose):
    return planner.make_transform(pose["xyz"], pose["rpy"])


def toy_radii_from_log():
    """按测例顺序取日志里“半对角线 x m”；缺日志时返回空列表。"""
    log_path = ENTRY_ROOT / "assets/q3.log"
    if not log_path.is_file():
        return []
    text = log_path.read_text(encoding="utf-8", errors="ignore")
    return [float(value) for value in re.findall(r"半对角线 ([\d.]+) m", text)]


def find_scene():
    """与 touch_toy.py 相同的定位顺序：环境变量 → 作品目录逐级向上找 sim/。"""
    env = os.environ.get("BAJIE_SIM_ROOT")
    candidates = [Path(env).expanduser() / "scene/abr_scene.usd"] if env else []
    candidates += [base / "sim/scene/abr_scene.usd" for base in (ENTRY_ROOT, *ENTRY_ROOT.parents)]
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    return None


def find_isaac_python():
    """Isaac 的 python.sh：环境变量优先，其次本机默认安装位置。"""
    env = os.environ.get("ISAAC_PYTHON")
    candidates = [Path(env).expanduser()] if env else []
    candidates += [
        Path("/mnt/hbotdata/tmp/isaac-sim/python.sh"),
        Path.home() / "isaac-sim/python.sh",
    ]
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    return None


def main():
    problems = []
    records = load_jsonl(ENTRY_ROOT / "inputs/q3_demo_input.jsonl")
    answers = load_jsonl(ENTRY_ROOT / "answers/q3_demo_output.jsonl")

    if len(records) != 3:
        problems.append(f"输入应有 3 行，实际 {len(records)} 行")
    if len(answers) != 3:
        problems.append(f"答案应有 3 行，实际 {len(answers)} 行")
    if [r["id"] for r in records] != [a["id"] for a in answers]:
        problems.append("答案 id 与输入不一致")
    if problems:
        return fail("；".join(problems))

    chain = planner.load_robot_chain()
    limits = planner.load_limits()
    print(f"ENV   Python {platform.python_version()}，numpy {np.__version__}")
    print(f"ENV   关节链 {len(chain)} 个关节加载成功")
    scene = find_scene()
    if scene is not None:
        print(f"ENV   场景 {scene}")
    else:
        print("WARN  未找到 abr_scene.usd；离线自检不受影响，复现需用 --scene 指定")
    isaac_python = find_isaac_python()
    if isaac_python is not None:
        print(f"ENV   Isaac python {isaac_python}")
    else:
        print("WARN  未找到 Isaac python.sh；离线自检不受影响，复现需先安装 Isaac Sim 4.5")
    radii = toy_radii_from_log()
    if len(radii) != 3:
        print("WARN  日志缺少三行玩具包围盒，距离判据回退到 3 厘米")

    for index, (record, answer) in enumerate(zip(records, answers)):
        case = answer["id"]
        for key in ("final_joint_state", "objects_link_in_base_footprint", "scene_object_in_objects_link"):
            if key not in answer:
                problems.append(f"{case} 缺少字段 {key}")
        if problems:
            continue

        state = answer["final_joint_state"]
        if set(state) != set(planner.JOINTS):
            problems.append(f"{case} 关节不是完整 10 个")
            continue
        bad = [name for name, value in state.items() if not limits[name][0] <= value <= limits[name][1]]
        if bad:
            problems.append(f"{case} 关节越界：{bad}")

        # FK 复核：答案里的 objects_link 位姿必须能由最终关节重算出来。
        fk = planner.fk_objects_link(chain, state)
        answer_matrix = pose_matrix(answer["objects_link_in_base_footprint"])
        fk_error = float(np.linalg.norm(fk[:3, 3] - answer_matrix[:3, 3]))
        rot_error = float(np.linalg.norm(fk[:3, :3] - answer_matrix[:3, :3]))
        if fk_error > 1e-5 or rot_error > 1e-4:
            problems.append(f"{case} FK 重算不一致：位置 {fk_error:.6f} m，旋转 {rot_error:.6f}")

        # 逆变换闭环：objects_link 位姿 × 玩具在 objects_link 的位姿，应回到玩具在底盘系的位姿。
        # 输入给的是玩具初始位姿，而答案里的玩具位姿是触碰后的；允许“玩具位移 + 2 毫米”的差。
        expected = planner.toy_transform_in_base(
            record["base_footprint_in_world"], record["scene_object_in_world"]
        )
        reconstructed = answer_matrix @ pose_matrix(answer["scene_object_in_objects_link"])
        loop_error = float(np.linalg.norm(expected[:3, 3] - reconstructed[:3, 3]))
        loop_limit = float(answer["toy_displacement_m"]) + 0.002
        if loop_error > loop_limit:
            problems.append(
                f"{case} 玩具位姿逆变换不闭环：{loop_error:.6f} m（允许 {loop_limit:.6f} m）"
            )

        # 输入口径的闭环：这一份按题面输入位姿算，必须回到输入（恐龙资产 -90° 也包含在内）。
        # 底盘用的是仿真中实测位姿，与输入标称差约 0.8 毫米，所以位置容差取 2 毫米。
        input_error = None
        if "scene_object_in_objects_link_from_input" in answer:
            from_input = answer_matrix @ pose_matrix(answer["scene_object_in_objects_link_from_input"])
            input_error = float(np.linalg.norm(expected[:3, 3] - from_input[:3, 3]))
            input_rot = float(np.linalg.norm(expected[:3, :3] - from_input[:3, :3]))
            if input_error > 2e-3 or input_rot > 2e-3:
                problems.append(
                    f"{case} 按输入口径的玩具位姿不闭环：位置 {input_error:.6f} m，旋转 {input_rot:.6f}"
                )

        # 物理证据：接触标志、距离兜底与力矩/位移至少一项成立。
        distance = float(answer["touch_distance_m"])
        effort = float(answer["finger_effort_nm"])
        displacement = float(answer["toy_displacement_m"])
        distance_limit = max(DISTANCE_FLOOR, radii[index]) if len(radii) == 3 else DISTANCE_FLOOR
        if not answer["contact_detected"]:
            problems.append(f"{case} contact_detected 为假")
        if distance > distance_limit + 1e-3:
            problems.append(f"{case} 距离 {distance:.4f} m 超过 {distance_limit:.4f} m")
        if effort < 0.15 and displacement < 0.005 and distance > DISTANCE_FLOOR:
            problems.append(f"{case} 力矩 {effort:.3f} N·m、位移 {displacement * 1000:.1f} mm 都不足以证明接触")

        print(
            f"OK    {case}: 距离 {distance:.4f} m（上限 {distance_limit:.4f}），"
            f"力矩 {effort:.3f} N·m，位移 {displacement * 1000:.1f} mm，"
            f"FK 差 {fk_error:.6f} m，闭环差 {loop_error:.6f} m"
            + (f"，输入口径差 {input_error:.6f} m" if input_error is not None else "")
        )

    log_path = ENTRY_ROOT / "assets/q3.log"
    if not log_path.is_file():
        problems.append("缺少 assets/q3.log")
    else:
        log_text = log_path.read_text(encoding="utf-8", errors="ignore")
        if log_text.count("完成：") != 3 or log_text.count("PASS") < 3:
            problems.append("日志里不是三次完成/PASS")
        if re.search(r"error|warning", log_text, re.IGNORECASE):
            problems.append("日志含 error/warning")

    video_path = ENTRY_ROOT / "video/q3_touch.mp4"
    if not video_path.is_file() or video_path.stat().st_size < MIN_VIDEO_BYTES:
        problems.append("缺少视频或视频过小")

    if problems:
        return fail("；".join(problems))
    print("PASS  题三答案、日志与视频自检通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
