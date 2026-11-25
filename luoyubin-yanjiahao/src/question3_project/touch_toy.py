#!/usr/bin/env python3
"""题三：在 Isaac Sim 中从指定初始关节状态出发，触碰三个玩具。

流程（每个测例）：
1. 置入输入给定的 10 关节初值，读回并打印误差；
2. 由底盘/玩具世界位姿求玩具在底盘系的坐标；
3. 规划上空预抓取点与玩具中心两个目标，阻尼最小二乘 IK 求关节解；
4. 插值下发关节目标（抬升并转向 → 下降到预抓取 → 下降到玩具），逐段打印夹爪—玩具距离；
5. 下降途中用“触底回升 / 指关节力矩 / 玩具位移”判定触碰，未触底时再合拢两指；
6. 读回最终关节，写一行 JSONL。

代码按职责拆分在本目录：
- `config.py`       常量（prim、抓取参数、帧数、视频与接触阈值）
- `input_io.py`     场景定位、输入校验、环境证据、测例过滤
- `contact.py`      触碰判定口径与 PhysX 接触报告订阅
- `sim_runtime.py`  Isaac Sim 启动、物理推进、底盘锁定、录像
- `arm.py`          关节动作原语（插值、过渡、下降、合拢）
- `planning.py`     目标点选择、多初值 IK、干跑预检
- `case_runner.py`  单个测例的执行与答案组装
本文件只解析参数、串起上面这些模块。

运行（先装好 Isaac Sim 4.5 与 ROS 2 Humble）：
    source /opt/ros/humble/setup.bash
    /mnt/hbotdata/tmp/isaac-sim/python.sh \
        src/question3_project/touch_toy.py \
        --scene /home/yjh/bajie-competition/sim/scene/abr_scene.usd \
        --input inputs/q3_demo_input.jsonl \
        --output answers/q3_demo_output.jsonl \
        --headless
加 `--video assets/q3_touch.mp4` 可在无窗口模式下用独立相机录出触碰视频。
"""
import argparse
import json
import sys
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parent
ENTRY_ROOT = PROJECT_DIR.parents[1]
sys.path.insert(0, str(PROJECT_DIR))

import planner  # noqa: E402
from arm import Arm  # noqa: E402
from case_runner import run_case  # noqa: E402
from contact import decide_touch  # noqa: E402,F401  （保留给回归测试导入）
from input_io import (  # noqa: E402
    environment_lines,
    find_scene,
    load_records,
    select_records,
)
from planning import dry_run  # noqa: E402
from sim_runtime import Sim  # noqa: E402


def parse_args():
    parser = argparse.ArgumentParser(description="题三：Isaac Sim 中触碰三个玩具")
    parser.add_argument("--scene", default=None, help="abr_scene.usd 路径")
    parser.add_argument("--input", default=str(ENTRY_ROOT / "inputs/q3_demo_input.jsonl"))
    parser.add_argument(
        "--output",
        default=None,
        help="答案路径；缺省写 answers/q3_demo_output.jsonl，--only 时缺省不写文件",
    )
    parser.add_argument("--urdf-root", default=None, help="官方 URDF 目录（缺省自动查找）")
    parser.add_argument("--headless", action="store_true", help="无窗口运行（生成答案用）")
    parser.add_argument("--video", default=None, help="同时录制视口视频到该路径（.mp4）")
    parser.add_argument("--env-only", action="store_true", help="只打印环境证据后退出，不启动仿真")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="不启动 Isaac，只用输入位姿做规划预检（目标点、IK 残差、是否穿地）",
    )
    parser.add_argument(
        "--only",
        nargs="+",
        default=None,
        metavar="ID_OR_NAME",
        help="只跑指定测例，可写 id 或玩具名（如 touch_toy_2 rubik_cube）",
    )
    parser.add_argument(
        "--finger-gap",
        type=float,
        default=0.0,
        metavar="RAD",
        help="手指闭合终点的指关节角（弧度），默认 0；留缝会削弱夹持力矩，正式答案保持 0",
    )
    parser.add_argument(
        "--hold-sec",
        type=float,
        default=None,
        help="跑完后保持窗口的秒数；缺省图形模式 90 秒、无窗口模式 0 秒",
    )
    args, _ = parser.parse_known_args()
    # Kit 会解析 sys.argv，先把自定义参数摘掉，避免误判。
    sys.argv = [sys.argv[0]]
    return args


def write_answers(results, output):
    path = Path(output or (ENTRY_ROOT / "answers/q3_demo_output.jsonl"))
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        for item in results:
            handle.write(json.dumps(item, ensure_ascii=False) + "\n")
    print(f"\n已写入 {len(results)} 行 -> {path}", flush=True)


def print_summary(results):
    print("\n结果汇总：", flush=True)
    for item in results:
        sensor = "有" if item["physx_contact"] else "无"
        print(
            f"  {item['id']}: 最终距离 {item['touch_distance_m']:.4f} m，"
            f"接触 {item['contact_detected']}，PhysX 接触报告 {sensor}，"
            f"指力矩 {item['finger_effort_nm']:.3f} N·m，"
            f"玩具位移 {item['toy_displacement_m'] * 1000:.1f} mm",
            flush=True,
        )


def main():
    args = parse_args()

    for line in environment_lines():
        print(line, flush=True)
    if args.env_only:
        return

    records = select_records(load_records(args.input), args.only)
    chain = planner.load_robot_chain(args.urdf_root)
    limits = planner.load_limits()
    if args.dry_run:
        sys.exit(dry_run(records, chain, limits))

    scene_path = find_scene(args.scene)
    print("场景：", scene_path, flush=True)
    print("关节链：", " → ".join(joint.name for joint in chain), flush=True)

    sim = Sim(scene_path, headless=args.headless, video=args.video)
    arm = Arm(sim, chain, limits)
    results = [run_case(arm, record, index, args.finger_gap) for index, record in enumerate(records)]

    if args.only and args.output is None:
        print(f"\n[SKIP] 只跑了 {len(results)} 个测例（--only），未写答案；需要写请显式指定 --output", flush=True)
    else:
        write_answers(results, args.output)

    sim.release_video()
    print_summary(results)

    # 图形模式下保持窗口一段时间，方便看清最后一个玩具的最终姿态；
    # 无窗口模式立即退出，避免影响批量运行。
    hold_sec = args.hold_sec if args.hold_sec is not None else (0 if args.headless else 90)
    sim.close(hold_sec)


if __name__ == "__main__":
    main()
