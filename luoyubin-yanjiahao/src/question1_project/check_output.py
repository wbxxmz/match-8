#!/usr/bin/env python3
"""题一答案自检：逐行校验答案是否符合官方题面要求。

检查项：行数与 id 对应、五段顺序、每帧 10 个关节、
数值在官方限位内、夹爪两段的开合值、final 与末帧一致。

用法（工作目录任意）：
    python3 src/question1_project/check_output.py
"""
import argparse
import json
from pathlib import Path

JOINTS = [
    "lift_joint", "pitch_joint",
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
    "arm_finger_joint1", "arm_finger_joint2",
]
SEGMENTS = ["lift", "head", "arm", "gripper_open", "gripper_close"]
FINGERS = ["arm_finger_joint1", "arm_finger_joint2"]

HERE = Path(__file__).resolve().parent
BASE = HERE.parent.parent


def load_jsonl(path):
    rows = []
    for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError as exc:
            raise SystemExit(f"[FAIL] {path.name} 第 {number} 行不是合法 JSON：{exc}")
    if not rows:
        raise SystemExit(f"[FAIL] {path} 没有任何有效行")
    return rows


def _arm_part(state):
    """去掉两个手指关节，只看升降、头部与机械臂。"""
    return {joint: value for joint, value in state.items() if joint not in FINGERS}


def check_record(record, source, limits):
    """返回该行的问题列表，空列表表示通过。"""
    problems = []

    def fail(message):
        problems.append(message)

    if record.get("id") != source.get("id"):
        fail(f"id 不一致：输出 {record.get('id')!r}，输入 {source.get('id')!r}")
    if record.get("step") != source.get("step"):
        fail(f"step 不一致：输出 {record.get('step')!r}，输入 {source.get('step')!r}")

    keyframes = record.get("keyframes")
    if not isinstance(keyframes, list) or len(keyframes) != len(SEGMENTS):
        fail(f"keyframes 应为 {len(SEGMENTS)} 帧，实际 {len(keyframes) if isinstance(keyframes, list) else '非列表'}")
        return problems

    names = [frame.get("name") for frame in keyframes]
    if names != SEGMENTS:
        fail(f"keyframes 顺序错误：{names}")

    for frame in keyframes:
        name = frame.get("name")
        state = frame.get("joint_state")
        if not isinstance(state, dict):
            fail(f"{name} 缺少 joint_state")
            continue
        missing = [joint for joint in JOINTS if joint not in state]
        extra = [joint for joint in state if joint not in JOINTS]
        if missing:
            fail(f"{name} 缺少关节：{missing}")
        if extra:
            fail(f"{name} 多出未知关节：{extra}")
        for joint, value in state.items():
            bound = limits.get(joint)
            if bound is None:
                continue
            if not isinstance(value, (int, float)):
                fail(f"{name}.{joint} 不是数值：{value!r}")
            elif value < bound["lower"] - 1e-9 or value > bound["upper"] + 1e-9:
                fail(f"{name}.{joint}={value} 超出限位 [{bound['lower']}, {bound['upper']}]")

    if not problems:
        lift = keyframes[0]["joint_state"]["lift_joint"]
        if lift <= 0:
            fail(f"lift 帧的升降值应大于 0，实际 {lift}")
        opened = keyframes[3]["joint_state"]
        closed = keyframes[4]["joint_state"]
        finger_upper = limits["arm_finger_joint1"]["upper"]
        for finger in FINGERS:
            if abs(opened[finger] - finger_upper) > 1e-6:
                fail(f"gripper_open.{finger} 应为 {finger_upper}，实际 {opened[finger]}")
            if abs(closed[finger]) > 1e-9:
                fail(f"gripper_close.{finger} 应为 0，实际 {closed[finger]}")
        if _arm_part(closed) != _arm_part(opened):
            fail("gripper_close 关键帧不应再改动机械臂姿态")

    final = record.get("final_joint_state")
    if final != keyframes[-1]["joint_state"]:
        fail("final_joint_state 与最后一帧不一致")

    return problems


def main():
    parser = argparse.ArgumentParser(description="题一答案自检")
    parser.add_argument(
        "--input",
        default=str(BASE / "inputs" / "q1_demo_input.jsonl"),
        help="输入 jsonl 路径",
    )
    parser.add_argument(
        "--output",
        default=str(BASE / "answers" / "q1_demo_output.jsonl"),
        help="答案 jsonl 路径",
    )
    parser.add_argument(
        "--limits",
        default=str(HERE / "config" / "joint_limits.json"),
        help="关节限位文件",
    )
    args = parser.parse_args()

    limits = json.loads(Path(args.limits).read_text(encoding="utf-8"))
    sources = load_jsonl(Path(args.input))
    answers = load_jsonl(Path(args.output))

    if len(answers) != len(sources):
        raise SystemExit(f"[FAIL] 答案 {len(answers)} 行，输入 {len(sources)} 行，数量不一致")

    failed = 0
    for index, (record, source) in enumerate(zip(answers, sources), 1):
        problems = check_record(record, source, limits)
        if problems:
            failed += 1
            print(f"[FAIL] 第 {index} 行 {record.get('id')!r}：")
            for problem in problems:
                print(f"       - {problem}")
        else:
            print(f"[PASS] 第 {index} 行 {record.get('id')!r}：5 帧顺序、10 关节、限位与最终态均一致")

    if failed:
        print(f"自检失败：{failed}/{len(answers)} 行有问题")
        return 1
    print(f"自检通过：{len(answers)} 行答案全部符合题面要求")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
