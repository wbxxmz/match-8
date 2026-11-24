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
