#!/usr/bin/env python3
"""解析官方 URDF，抽取 base_footprint → objects_link 的关节链。

流程：先用 xacro 展开 `urdf/bajie_robot.urdf.xacro`，再用标准库解析。
xacro 里用了 $(find bajie_robot) 依赖 ament 资源索引，这里在 /tmp 下
临时建一个最小前缀并软链到 urdf 目录，展开完成后清理。

关节原点、轴向全部从 URDF 读取，不硬编码数值；固定关节也保留在链上，
因为它们同样贡献变换。
"""
import os
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

PACKAGE = "bajie_robot"
XACRO_RELATIVE = Path("urdf") / "bajie_robot.urdf.xacro"
ROOT_LINK = "base_footprint"
TIP_LINK = "objects_link"
# 未 source ROS 时直接找 Humble 的 xacro 可执行文件。
XACRO_FALLBACKS = ("/opt/ros/humble/bin/xacro",)


@dataclass(frozen=True)
class JointSpec:
    """链上一个关节的静态参数。"""

    name: str
    type: str
    xyz: tuple
    rpy: tuple
    axis: tuple

    @property
    def movable(self):
        return self.type in ("revolute", "continuous", "prismatic")


def _numbers(text, default, label):
    """把 URDF 属性里的空格分隔数值转成 3 元组，缺省时用 default。"""
    if text is None or not text.strip():
        return default
    try:
        values = tuple(float(item) for item in text.split())
    except ValueError as exc:
        raise ValueError(f"{label} 不是数值列表：{text!r}") from exc
    if len(values) != 3:
        raise ValueError(f"{label} 应为 3 个数值，实际 {len(values)} 个：{text!r}")
    return values


def find_xacro():
    """优先用 PATH 里的 xacro，退化到 ROS Humble 的绝对路径。"""
    path = shutil.which("xacro")
    if path:
        return path
    for candidate in XACRO_FALLBACKS:
        if Path(candidate).is_file():
            return candidate
    raise FileNotFoundError(
        "找不到 xacro。请先执行 `source /opt/ros/humble/setup.bash`，"
        "或安装 ros-humble-xacro。"
    )


def _ros_python_paths(xacro_path):
    """xacro 来自 ROS 安装时，返回它的 Python 包目录，保证未 source 也能导入。"""
    prefix = Path(xacro_path).resolve().parent.parent
    pyver = f"python{sys.version_info.major}.{sys.version_info.minor}"
    candidates = [
        prefix / "lib" / pyver / "site-packages",
        prefix / "local" / "lib" / pyver / "dist-packages",
    ]
    return [path for path in candidates if path.is_dir()]


def expand_urdf(urdf_root):
    """展开官方 xacro，返回 URDF 文本，package:// 已替换为本地 file://。"""
    urdf_root = Path(urdf_root).resolve()
    xacro_file = urdf_root / XACRO_RELATIVE
    if not xacro_file.is_file():
        raise FileNotFoundError(f"找不到官方模型入口: {xacro_file}")
    xacro_bin = find_xacro()
    with tempfile.TemporaryDirectory(prefix="q2_ament_") as tmp:
        prefix = Path(tmp)
        resource_dir = prefix / "share" / "ament_index" / "resource_index" / "packages"
        resource_dir.mkdir(parents=True)
        (resource_dir / PACKAGE).write_text("", encoding="utf-8")
        pkg_dir = prefix / "share" / PACKAGE
        os.symlink(urdf_root, pkg_dir)
        env = dict(os.environ)
        env["AMENT_PREFIX_PATH"] = f"{prefix}:{env.get('AMENT_PREFIX_PATH', '')}"
        site_packages = _ros_python_paths(xacro_bin)
        existing = env.get("PYTHONPATH", "").split(":")
        extra = [str(path) for path in site_packages if str(path) not in existing]
        if extra:
            env["PYTHONPATH"] = ":".join(extra + [env.get("PYTHONPATH", "")]).rstrip(":")
        try:
            result = subprocess.run(
                [xacro_bin, str(xacro_file)],
                check=True,
                capture_output=True,
                text=True,
                env=env,
            )
        except subprocess.CalledProcessError as exc:
            detail = (exc.stderr or "").strip()
            raise RuntimeError(f"xacro 展开失败（退出码 {exc.returncode}）：\n{detail}") from exc
    text = result.stdout
    if not text.strip():
        raise RuntimeError(f"xacro 展开结果为空：{xacro_file}")
    return text.replace(f"package://{PACKAGE}/", f"file://{urdf_root}/")


def build_chain(root, root_link=ROOT_LINK, tip_link=TIP_LINK):
    """从解析后的 URDF 根节点，沿父关节从 tip_link 上溯到 root_link。"""
    by_child = {}
    for joint in root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            raise ValueError(f"关节 {joint.get('name')!r} 缺少 parent 或 child")
        link = child.get("link")
        if link in by_child:
            raise ValueError(f"连杆 {link!r} 有多个父关节，URDF 不是树")
        by_child[link] = joint

    chain = []
    link = tip_link
    while link != root_link:
        joint = by_child.get(link)
        if joint is None:
            raise ValueError(f"从 {tip_link!r} 上溯找不到 {root_link!r}：断在 {link!r}")
        origin = joint.find("origin")
        origin_xyz = origin.get("xyz") if origin is not None else None
        origin_rpy = origin.get("rpy") if origin is not None else None
        axis_el = joint.find("axis")
        axis_text = axis_el.get("xyz") if axis_el is not None else None
        axis = _numbers(axis_text, (1.0, 0.0, 0.0), f"关节 {joint.get('name')} 的 axis")
        joint_type = joint.get("type")
        if joint_type in ("revolute", "continuous", "prismatic") and all(
            abs(value) < 1e-12 for value in axis
        ):
            raise ValueError(f"运动关节 {joint.get('name')!r} 的 axis 为零向量")
        chain.append(
            JointSpec(
                name=joint.get("name"),
                type=joint_type,
                xyz=_numbers(origin_xyz, (0.0, 0.0, 0.0), f"关节 {joint.get('name')} 的 origin xyz"),
                rpy=_numbers(origin_rpy, (0.0, 0.0, 0.0), f"关节 {joint.get('name')} 的 origin rpy"),
                axis=axis,
            )
        )
        link = joint.find("parent").get("link")
    chain.reverse()
    return chain


def load_chain(urdf_root, root_link=ROOT_LINK, tip_link=TIP_LINK):
    """展开官方模型并返回 base_footprint → objects_link 的关节链。"""
    root = ET.fromstring(expand_urdf(urdf_root))
    return build_chain(root, root_link, tip_link)
