#!/usr/bin/env python3
"""题二：正向运动学与坐标变换。

读 `q2_demo_input.jsonl`，逐行算 objects_link 相对 base_footprint 的位姿，
再把场景物体从底盘系变换到夹爪系，写 `q2_demo_output.jsonl`。

运行（作品根目录）：
    python3 src/question2_project/run_q2.py \
        --input inputs/q2_demo_input.jsonl \
        --output answers/q2_demo_output.jsonl
"""
import argparse
import json
import math
import os
from pathlib import Path

from fk import objects_link_transform
from transforms import invert_transform, make_transform, pose_from_transform
from urdf_chain import XACRO_RELATIVE, load_chain

# 作品根目录：src/question2_project/run_q2.py 往上 3 层是 luoyubin-yanjiahao。
PROJECT_DIR = Path(__file__).resolve().parent
ENTRY_ROOT = PROJECT_DIR.parents[1]
# 官方 xacro 源码的内置副本（不含 mesh，约 45 KB），官方资源仓库不在时兜底。
BUNDLED_URDF_ROOT = PROJECT_DIR / "model"
URDF_ENV = "BAJIE_URDF_ROOT"
DECIMALS = 6


def environment_lines():
    """打印复现所需的环境证据；拿不到的字段写 unknown，不影响正常流程。"""
    import platform
    import subprocess

    def first(command):
        try:
            out = subprocess.run(command, capture_output=True, text=True, timeout=5).stdout.strip()
            return out.splitlines()[0] if out else "unknown"
        except Exception:  # 没有 git 也不报错
            return "unknown"

    import numpy

    commit = first(["git", "-C", str(ENTRY_ROOT), "rev-parse", "--short", "HEAD"])
    return [
        f"环境  OS {platform.platform()}",
        f"环境  Python {platform.python_version()}  numpy {numpy.__version__}",
        f"环境  ROS_DISTRO {os.environ.get('ROS_DISTRO', 'unset')}  "
        f"ROS_DOMAIN_ID {os.environ.get('ROS_DOMAIN_ID', 'unset')}",
        f"环境  git {commit}",
    ]


def find_urdf_root(explicit=None):
    """定位官方 URDF 资源目录：参数 > 环境变量 > 外部 urdf/ > 作品内置副本。"""
    if explicit:
        return Path(explicit).expanduser().resolve()
    env = os.environ.get(URDF_ENV)
    if env:
        return Path(env).expanduser().resolve()
    tried = []
    for base in (ENTRY_ROOT, *ENTRY_ROOT.parents):
        candidate = base / "urdf"
        tried.append(str(candidate))
        if (candidate / XACRO_RELATIVE).is_file():
            return candidate
    if (BUNDLED_URDF_ROOT / XACRO_RELATIVE).is_file():
        return BUNDLED_URDF_ROOT
    raise FileNotFoundError(
        "找不到官方 URDF 资源目录，已尝试：\n  "
        + "\n  ".join(tried)
        + f"\n内置副本也缺失：{BUNDLED_URDF_ROOT}"
        + f"\n请用 --urdf-root 指定，或设置环境变量 {URDF_ENV}。"
    )


def round_number(value):
    """保留 6 位小数；把 -0.0 归一为 0.0，避免输出里出现负零。"""
    rounded = round(float(value), DECIMALS)
    return 0.0 if rounded == 0.0 else rounded


def round_pose(pose):
    return {key: [round_number(value) for value in values] for key, values in pose.items()}


def _vector(record, key, path, number):
    value = record.get(key)
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{path} 第 {number} 行字段 {key!r} 应为 3 个数值的数组")
    result = []
    for item in value:
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise ValueError(f"{path} 第 {number} 行字段 {key!r} 含非数值：{item!r}")
        item = float(item)
        if not math.isfinite(item):
            raise ValueError(f"{path} 第 {number} 行字段 {key!r} 含非有限数值：{item!r}")
        result.append(item)
    return result


def load_records(path):
    """逐行读取并校验输入，出错时指出文件名与行号。"""
    records = []
    for number, raw in enumerate(Path(path).read_text(encoding="utf-8").splitlines(), 1):
        if not raw.strip():
            continue
        try:
            record = json.loads(raw)
        except json.JSONDecodeError as exc:
            raise ValueError(f"{path} 第 {number} 行不是合法 JSON：{exc}") from exc
        if not isinstance(record, dict):
            raise ValueError(f"{path} 第 {number} 行不是 JSON 对象")
        for key in ("id", "joint_state", "scene_object_in_base_footprint"):
            if key not in record:
                raise ValueError(f"{path} 第 {number} 行缺少字段 {key!r}")
        scene = record["scene_object_in_base_footprint"]
        if not isinstance(scene, dict):
            raise ValueError(f"{path} 第 {number} 行字段 'scene_object_in_base_footprint' 应为对象")
        _vector(scene, "xyz", path, number)
        _vector(scene, "rpy", path, number)
        records.append(record)
    if not records:
        raise ValueError(f"{path} 没有任何有效行")
    return records


def run_record(record, chain):
    """单个测例：正向运动学 + 把场景物体变换到 objects_link。"""
    objects_in_base = objects_link_transform(chain, record["joint_state"])
    scene = record["scene_object_in_base_footprint"]
    scene_in_base = make_transform(scene["xyz"], scene["rpy"])
    scene_in_objects = invert_transform(objects_in_base) @ scene_in_base
    return {
        "id": record["id"],
        "objects_link_in_base_footprint": round_pose(pose_from_transform(objects_in_base)),
        "scene_object_in_objects_link": round_pose(pose_from_transform(scene_in_objects)),
    }


def main():
    parser = argparse.ArgumentParser(description="题二：正向运动学与坐标变换")
    parser.add_argument("--input", required=True, help="输入 jsonl 路径")
    parser.add_argument("--output", required=True, help="输出 jsonl 路径")
    parser.add_argument(
        "--urdf-root",
        default=None,
        help=(
            "官方 URDF 资源目录（含 urdf/bajie_robot.urdf.xacro）；"
            f"缺省时读环境变量 {URDF_ENV}，再逐级向上查找 urdf/，最后用作品内置副本"
        ),
    )
    args = parser.parse_args()

    records = load_records(args.input)
    urdf_root = find_urdf_root(args.urdf_root)
    for line in environment_lines():
        print(line, flush=True)
    print(f"环境  模型 {urdf_root}", flush=True)
    chain = load_chain(urdf_root)
    print("关节链：" + " → ".join(joint.name for joint in chain), flush=True)

    results = [run_record(record, chain) for record in records]

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", encoding="utf-8") as fh:
        for item in results:
            fh.write(json.dumps(item, ensure_ascii=False) + "\n")
    print(f"已写入 {len(results)} 行 -> {out}")


if __name__ == "__main__":
    main()
