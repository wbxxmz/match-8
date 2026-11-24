#!/usr/bin/env python3
"""把官方 xacro 展开成 urdf 文本，并把 package:// 路径改成本地 file://。

xacro 里用了 $(find bajie_robot) 依赖 ament 资源索引，这里在 /tmp 下
临时建一个最小前缀并软链到 urdf 目录，展开完成后清理。
"""
import os
import subprocess
import tempfile
from pathlib import Path

PACKAGE = "bajie_robot"
XACRO = Path("urdf/bajie_robot.urdf.xacro")


def find_urdf_root(explicit=None, search_bases=()):
    """定位官方 URDF 目录：显式参数 > BAJIE_URDF_ROOT > 逐级向上的 urdf/。

    找不到时抛 FileNotFoundError 并列出试过的路径，避免直接崩在临时聚合前缀里。
    """
    candidates = []
    if explicit:
        candidates.append(Path(explicit).expanduser())
    env = os.environ.get("BAJIE_URDF_ROOT")
    if env:
        candidates.append(Path(env).expanduser())
    for base in search_bases:
        candidates.append(Path(base) / "urdf")
    tried = []
    for candidate in candidates:
        candidate = candidate.resolve()
        tried.append(str(candidate))
        if (candidate / XACRO).is_file():
            return candidate
    raise FileNotFoundError(
        "找不到官方 URDF 目录（其中需有 "
        + str(XACRO)
        + "）。已尝试：\n  "
        + "\n  ".join(tried)
        + "\n请设置环境变量 BAJIE_URDF_ROOT 指向官方 urdf 仓库根目录，例如：\n"
        "  export BAJIE_URDF_ROOT=/path/to/bajie_urdf_repo"
    )


def expand_urdf(urdf_root):
    urdf_root = Path(urdf_root).resolve()
    xacro_file = urdf_root / XACRO
    if not xacro_file.is_file():
        raise FileNotFoundError(f"找不到官方模型入口: {xacro_file}")
    with tempfile.TemporaryDirectory(prefix="q1_ament_") as tmp:
        prefix = Path(tmp)
        (prefix / "share" / "ament_index" / "resource_index" / "packages").mkdir(parents=True)
        (prefix / "share" / "ament_index" / "resource_index" / "packages" / PACKAGE).write_text("")
        pkg_dir = prefix / "share" / PACKAGE
        pkg_dir.parent.mkdir(parents=True, exist_ok=True)
        os.symlink(urdf_root, pkg_dir)
        env = dict(os.environ)
        env["AMENT_PREFIX_PATH"] = f"{prefix}:{env.get('AMENT_PREFIX_PATH', '')}"
        result = subprocess.run(
            ["xacro", str(xacro_file)],
            check=True,
            capture_output=True,
            text=True,
            env=env,
        )
    text = result.stdout
    return text.replace(f"package://{PACKAGE}/", f"file://{urdf_root}/")
