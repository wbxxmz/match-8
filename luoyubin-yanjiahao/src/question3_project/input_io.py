#!/usr/bin/env python3
"""题三输入输出：定位场景、读取并校验 JSONL、环境证据与测例过滤。"""
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

import planner
from config import ENTRY_ROOT, SCENE_ENV, TOY_PRIMS


def find_scene(explicit=None):
    """定位 abr_scene.usd：参数 > 环境变量 > 从作品目录逐级向上找 sim/。"""
    if explicit:
        return Path(explicit).expanduser().resolve()
    env = os.environ.get(SCENE_ENV)
    if env:
        return Path(env).expanduser().resolve() / "scene" / "abr_scene.usd"
    tried = []
    for base in (ENTRY_ROOT, *ENTRY_ROOT.parents):
        candidate = base / "sim" / "scene" / "abr_scene.usd"
        tried.append(str(candidate))
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(
        "找不到 abr_scene.usd，已尝试：\n  " + "\n  ".join(tried)
        + f"\n请用 --scene 指定，或设置环境变量 {SCENE_ENV} 指向 sim 仓库根目录。"
    )


def load_records(path):
    """逐行读取输入，校验 id / 关节 / 位姿字段，出错时报行号。"""
    records = []
    for number, raw in enumerate(Path(path).read_text(encoding="utf-8").splitlines(), 1):
        if not raw.strip():
            continue
        try:
            record = json.loads(raw)
        except json.JSONDecodeError as exc:
            raise ValueError(f"{path} 第 {number} 行不是合法 JSON：{exc}") from exc
        for key in ("id", "joint_state", "base_footprint_in_world", "scene_object_in_world", "name"):
            if key not in record:
                raise ValueError(f"{path} 第 {number} 行缺少字段 {key!r}")
        if record["name"] not in TOY_PRIMS:
            raise ValueError(f"{path} 第 {number} 行玩具名未知：{record['name']!r}")
        try:
            planner.full_state(record["joint_state"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"{path} 第 {number} 行 joint_state 非法：{exc}") from exc
        for key in ("base_footprint_in_world", "scene_object_in_world"):
            check_pose(record[key], key, path, number)
        records.append(record)
    if not records:
        raise ValueError(f"{path} 没有任何有效行")
    return records


def check_pose(pose, key, path, number):
    """位姿必须是 {xyz:[3], rpy:[3]}，且每个数有限；否则报行号。"""
    if not isinstance(pose, dict):
        raise ValueError(f"{path} 第 {number} 行字段 {key!r} 应为对象")
    for field in ("xyz", "rpy"):
        values = pose.get(field)
        if not isinstance(values, list) or len(values) != 3:
            raise ValueError(f"{path} 第 {number} 行字段 {key}.{field} 应为 3 个数值的数组")
        for item in values:
            if isinstance(item, bool) or not isinstance(item, (int, float)):
                raise ValueError(f"{path} 第 {number} 行字段 {key}.{field} 含非数值：{item!r}")
            if not math.isfinite(item):
                raise ValueError(f"{path} 第 {number} 行字段 {key}.{field} 含非有限数值：{item!r}")


def make_pose(transform):
    """4×4 → {'xyz','rpy'}，供 planner 使用。"""
    return {"xyz": [float(v) for v in transform[:3, 3]], "rpy": planner.pose_in_parent(transform)["rpy"]}


def environment_lines():
    """打印复现所需的环境证据；拿不到的字段写 unknown，不影响正常流程。"""
    import platform
    import subprocess

    def first(command):
        try:
            out = subprocess.run(command, capture_output=True, text=True, timeout=5).stdout.strip()
            return out.splitlines()[0] if out else "unknown"
        except Exception:  # 没有 git / nvidia-smi 也不报错
            return "unknown"

    lines = [
        f"环境  OS {platform.platform()}",
        f"环境  Python {platform.python_version()}  numpy {np.__version__}",
        f"环境  ROS_DISTRO {os.environ.get('ROS_DISTRO', 'unset')}  "
        f"ROS_DOMAIN_ID {os.environ.get('ROS_DOMAIN_ID', 'unset')}",
        f"环境  git {first(['git', '-C', str(ENTRY_ROOT), 'rev-parse', '--short', 'HEAD'])}",
        f"环境  GPU {first(['nvidia-smi', '--query-gpu=name,driver_version', '--format=csv,noheader'])}",
    ]
    # Isaac Sim 安装目录带 VERSION 文件；从 python.sh 的 python 路径逐级向上找。
    for parent in Path(sys.executable).resolve().parents:
        version = parent / "VERSION"
        if version.is_file():
            lines.append(f"环境  Isaac Sim {version.read_text(encoding='utf-8').strip()}")
            break
    return lines


def select_records(records, only):
    """按 --only 过滤输入；id 与玩具名都能匹配，找不到就列出可选值。"""
    if not only:
        return records
    wanted = set(only)
    chosen = [
        record for record in records
        if record["id"] in wanted or record["name"] in wanted
    ]
    if not chosen:
        options = ", ".join(f"{record['id']}({record['name']})" for record in records)
        raise SystemExit(f"[FAIL] --only 没有匹配任何测例；可选：{options}")
    return chosen
