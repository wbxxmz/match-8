#!/usr/bin/env python3
"""题三：在 Isaac Sim 中从指定初始关节状态出发，触碰三个玩具。

流程（每个测例）：
1. 置入输入给定的 10 关节初值，读回并打印误差；
2. 由底盘/玩具世界位姿求玩具在底盘系的坐标；
3. 规划上空预抓取点与玩具中心两个目标，阻尼最小二乘 IK 求关节解；
4. 插值下发关节目标（抬升并转向 → 下降到预抓取 → 下降到玩具），逐段打印夹爪—玩具距离；
5. 下降途中用“触底回升 / 指关节力矩 / 玩具位移”判定触碰，未触底时再合拢两指；
6. 读回最终关节，写一行 JSONL。

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
import math
import os
import sys
from pathlib import Path

import numpy as np

PROJECT_DIR = Path(__file__).resolve().parent
ENTRY_ROOT = PROJECT_DIR.parents[1]
sys.path.insert(0, str(PROJECT_DIR))

import planner  # noqa: E402

# 官方场景里三个玩具的 prim；映射由世界坐标与输入位姿核对（差 < 2 cm）。
TOY_PRIMS = {
    "rubik_cube": "/World/company/toys/Rubik",
    "dinosaur": "/World/company/toys/toy_yellow_dragon",
    "soft_toy": "/World/company/toys/spiderman_grasp",
}
ROBOT_ROOT = "/World/abr_machine/robot_root"
BASE_PRIM = ROBOT_ROOT + "/base/base_footprint"
OBJECTS_PRIM = ROBOT_ROOT + "/gripper/objects_link"
SCENE_ENV = "BAJIE_SIM_ROOT"

TOUCH_DISTANCE = 0.03      # 距离判据下限（米）；实际阈值取它与玩具包络半径的较大者
PREGRASP_HEIGHT = 0.15     # 预抓取点抬高量（米）
WIDE_GRASP_TOYS = {"rubik_cube", "soft_toy"}  # 两指从两侧包住、不压顶面的玩具
WIDE_GRASP_CLEARANCE = {"rubik_cube": 0.005, "soft_toy": 0.02}  # 掌部参考点停在中心上方的高度（米）
WIDE_GRASP_AXIS = {"rubik_cube": (1.0, 0.0, 0.0), "soft_toy": (0.0, 1.0, 0.0)}  # 两指张开方向（底盘系）
SIDE_WIDE_TOYS = {"dinosaur"}     # 长条玩具：竖直下、两指沿短轴张开，从身体两侧包住
SIDE_WIDE_CLEARANCE = 0.02        # 侧抱时掌部参考点停在玩具中心上方的高度（米）
FRONT_GRASP_TOYS = set()          # 正面抓停用：40° 下倾会把轻质玩具推转 45°
FRONT_GRASP_PULLBACK = 0.12       # 正面抓的预抓取点沿接近方向后退的距离（米）
FRONT_GRASP_FACE_HEIGHT = 0.02    # 正面抓的指尖目标比玩具中心高出的量（米，对准面部）
# 绕竖直轴的抓取偏转：只换两指方位、不改接近方向。取值是离线对比各偏转下的
# 关节姿态后选的——避开手腕接近限位的解，否则肘部会扭成不自然的角度。
GRASP_YAW_DEG = {"rubik_cube": 0.0, "dinosaur": 0.0, "soft_toy": 0.0}
FINGER_OPEN = 0.5                 # 夹爪张开目标（弧度，略低于 URDF 上限 0.506，避免物理过冲越限）
TRANSFER_HEIGHT = 0.30     # 测例之间的转运高度（米），与单例内部转运一致
TRANSFER_MICRO_LIFT_FRAMES = 20   # 松指前带玩具微抬的帧数
TRANSFER_FINGER_FRAMES = 40       # 松指帧数（与闭合对称，减小对玩具的横向推力）
TRANSFER_LIFT_FRAMES = 40         # 竖直抬到转运高度的帧数
TRANSFER_BLEND_FRAMES = 70        # 在转运高度把姿态插值到下一例初值的帧数
TRANSFER_LIFT_DOWN_FRAMES = 30    # 姿态到位后单独下降的帧数
TRANSFER_SETTLE_FRAMES = 40       # 下降后保持目标命令、等物理收敛的上限帧数
TRANSFER_ALIGN_FRAMES = 30        # 收尾把实际关节位置分帧硬插到目标的帧数（消除静差瞬移）
EFFORT_CONTACT = 0.15      # 指关节力矩超过此值即认定真实接触（N·m）
TOY_DISPLACEMENT = 0.005   # 玩具被推离原位超过此值即认定真实接触（米）
DEEPEN_DEPTH = 0.01        # 未接触时每次继续下探的深度（米）
DEEPEN_STEPS = 3           # 最多下探次数（3 cm，避免目标点落到地面以下）
PHYSICS_STEPS_PER_FRAME = 2
TOY_SETTLE_STEPS = 30      # 玩具摆到题面位姿后等待物理收敛的步数（0.5 秒）

# 视频（独立相机；1280×720，每 4 个物理步渲染一帧，按 15 fps 写出即实时速度）
VIDEO_EYE = (10.55, 3.75, 2.55)      # 相机位置（世界系，拉远抬高，同时拍到抬升段与最近玩具）
VIDEO_TARGET = (12.90, 5.30, 0.35)   # 相机注视点（世界系，机器人中段与三个玩具之间）
VIDEO_WIDTH = 1280
VIDEO_HEIGHT = 720
VIDEO_FPS = 15
VIDEO_STRIDE = 4
VIDEO_CODEC = "mp4v"

# 接触报告：只认 PhysX 上报的真实碰撞对，作为“碰到”的客观证据。
CONTACT_MIN_IMPULSE = 1e-6   # 冲量下限（N·s），滤掉数值噪声
CONTACT_MAX_SEPARATION = 0.001  # 允许的最大分离距离（米），超过即视为未接触


def decide_touch(contact, sensor_event):
    """触碰判定：力学证据或 PhysX 接触报告任一成立即 PASS，距离不参与。

    距离近不等于碰到，所以 final 距离只作几何参考；答案里的 contact_detected
    也用同一个结果，保证运行、自检与答案三处口径一致。
    """
    return bool(contact) or sensor_event is not None


class ContactMonitor:
    """订阅 PhysX 接触报告，按当前测例记录目标玩具与意外碰撞。

    PhysX 只对挂了 PhysxContactReportAPI 的刚体上报接触。机器人各连杆与三个玩具都挂上，
    阈值设 0（默认阈值会漏掉轻触），回调里只保留“有机器人连杆参与”且冲量、分离距离达标的事件。
    """

    def __init__(self, stage, robot_root, toy_paths):
        import omni.physx
        from pxr import PhysicsSchemaTools, PhysxSchema, UsdPhysics

        self.robot_root = robot_root
        self.toy_paths = set(toy_paths)
        self.actor_path = PhysicsSchemaTools.intToSdfPath
        self.target_path = None
        self.target = None
        self.unexpected = []
        self.events = 0
        self.paused = False
        for prim in stage.Traverse():
            path = str(prim.GetPath())
            if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
            if path.startswith(robot_root + "/") or path in self.toy_paths:
                PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
        self.subscription = (
            omni.physx.get_physx_simulation_interface()
            .subscribe_contact_report_events(self.record)
        )

    def begin(self, toy_path):
        """一个测例开始前重置，只记该玩具的首次有效接触。"""
        self.target_path = toy_path
        self.target = None
        self.unexpected = []
        self.events = 0
        self.paused = False

    def pause(self):
        """暂停记录：测例之间的过渡动作不属于任何测例，不计入接触证据。

        过渡期间真实发生的扰动另有证据：每次过渡后都会打印三个玩具的位移。
        """
        self.paused = True

    def record(self, headers, data):
        if self.paused:
            return
        for header in headers:
            actors = [str(self.actor_path(actor)) for actor in (header.actor0, header.actor1)]
            if not any(actor.startswith(self.robot_root + "/") for actor in actors):
                continue
            # 轮子压地、底盘蹭地不是我们要的证据
            if any("CollisionPlane" in actor or "GroundPlane" in actor for actor in actors) and any(
                "/wheels/" in actor or actor.endswith("/base_link") for actor in actors
            ):
                continue
            intended = self.target_path in actors and any(
                actor.startswith(self.robot_root + "/gripper/") for actor in actors
            )
            other_toys = [a for a in actors if a in self.toy_paths and a != self.target_path]
            offset = header.contact_data_offset
            for index in range(offset, offset + header.num_contact_data):
                point = data[index]
                impulse = float(np.linalg.norm(point.impulse))
                separation = float(point.separation)
                if impulse <= CONTACT_MIN_IMPULSE or separation > CONTACT_MAX_SEPARATION:
                    continue
                self.events += 1
                event = {
                    "actors": actors,
                    "point_world_m": [round(float(v), 4) for v in point.position],
                    "impulse_ns": round(impulse, 6),
                    "separation_m": round(separation, 6),
                }
                if intended and self.target is None:
                    self.target = event
                if other_toys:
                    self.unexpected.append({"toy": other_toys[0], **event})
                break

    def stop(self):
        try:
            self.subscription.unsubscribe()
        except Exception:  # 关闭阶段 PhysX 可能已释放
            pass



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


def main():
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

    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720})
    print("ISAAC_OK", flush=True)

    import omni.usd
    from isaacsim.core.utils.extensions import enable_extension
    from pxr import Usd, UsdGeom

    enable_extension("isaacsim.ros2.bridge")
    for _ in range(5):
        simulation_app.update()

    context = omni.usd.get_context()
    context.open_stage(str(scene_path))
    stage = None
    for _ in range(1200):
        simulation_app.update()
        stage = context.get_stage()
        if stage is not None and stage.GetDefaultPrim() and stage.GetPrimAtPath(ROBOT_ROOT):
            break
    if stage is None or not stage.GetPrimAtPath(ROBOT_ROOT):
        raise RuntimeError("场景加载失败或缺少 robot_root")
    print("STAGE_OK", stage.GetDefaultPrim().GetPath(), flush=True)
    contact_monitor = ContactMonitor(stage, ROBOT_ROOT, TOY_PRIMS.values())
    print("CONTACT_REPORT_ON", len(TOY_PRIMS), "个玩具", flush=True)

    if not args.headless:
        # 图形窗口里把视口相机对准机器人工作区，否则默认视角看不到三个玩具。
        from isaacsim.core.utils.viewports import set_camera_view
        from omni.kit.viewport.utility import get_active_viewport

        set_camera_view(eye=VIDEO_EYE, target=VIDEO_TARGET)
        active = get_active_viewport()
        print("VIEWPORT", None if active is None else active.resolution, flush=True)

    from isaacsim.core.api import World
    from isaacsim.core.prims import Articulation, RigidPrim

    world = World()
    world.play()
    for _ in range(120):
        world.step(render=not args.headless)
    print("PLAY_OK", flush=True)

    # 视频：独立相机 + RGB 标注器。world.render() 只渲染、不推进物理，能同步取到画面。
    writer = None
    frame_count = 0
    if args.video:
        import cv2
        import omni.replicator.core as rep

        camera = rep.create.camera(position=VIDEO_EYE, look_at=VIDEO_TARGET, name="q3_video_camera")
        render_product = rep.create.render_product(camera, (VIDEO_WIDTH, VIDEO_HEIGHT))
        rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb.attach([render_product])
        writer = cv2.VideoWriter(
            str(args.video), cv2.VideoWriter_fourcc(*VIDEO_CODEC), VIDEO_FPS,
            (VIDEO_WIDTH, VIDEO_HEIGHT),
        )
        if not writer.isOpened():
            raise RuntimeError(f"无法创建视频文件：{args.video}")
        print("VIDEO", args.video, VIDEO_WIDTH, VIDEO_HEIGHT, flush=True)

    render_each_step = (not args.headless) or (writer is not None)

    def advance(steps=1):
        """推进物理；录视频时每 VIDEO_STRIDE 步渲染一帧写进视频（不额外推进物理）。

        图形模式下每步都要渲染，否则窗口停在第一帧、看不到机械臂动作。
        """
        nonlocal frame_count
        for _ in range(steps):
            world.step(render=render_each_step)
            lock_base()
            if writer is None:
                continue
            frame_count += 1
            if frame_count % VIDEO_STRIDE:
                continue
            world.render()
            data = rgb.get_data()
            if data is None or not np.asarray(data).size:
                continue
            image = np.asarray(data)[:, :, :3]
            if image.dtype != np.uint8:
                image = np.clip(image, 0, 255).astype(np.uint8)
            writer.write(cv2.cvtColor(image, cv2.COLOR_RGB2BGR))


    articulation = Articulation(BASE_PRIM)
    articulation.initialize()
    dof_names = list(articulation.dof_names)
    print("DOF", dof_names, flush=True)

    # 底盘锁定：这台机器人是轮式自由 articulation，机械臂伸展时的反作用会把整台车推走
    # （实测 1~33 厘米），答案位姿与真实位姿就错开了。该 Isaac 版本没有 fixRootJoint，
    # 所以在每个物理步之后把根位姿写回初始值，等价于把底盘钉在原地。
    try:
        root_position, root_orientation = articulation.get_world_poses()
        root_position = np.asarray(root_position, dtype=float).copy()
        root_orientation = np.asarray(root_orientation, dtype=float).copy()
    except Exception as exc:  # 拿不到根位姿就退回自由底盘，并在日志里说清
        root_position = root_orientation = None
        print(f"BASE_LOCK 失败：{exc}（底盘将自由移动）", flush=True)
    if root_position is not None:
        print("BASE_LOCK_ON", np.round(root_position.reshape(-1), 4).tolist(), flush=True)

    def lock_base():
        if root_position is None:
            return
        articulation.set_world_poses(root_position, root_orientation)

    def read_state():
        values = np.asarray(articulation.get_joint_positions(), dtype=float).reshape(-1)
        return {name: float(values[i]) for i, name in enumerate(dof_names)}

    def state_vector(state):
        values = np.asarray(articulation.get_joint_positions(), dtype=float).reshape(-1)
        for index, name in enumerate(dof_names):
            if name in state:
                values[index] = float(state[name])
        return values.reshape(1, -1)

    def drive(state, steps):
        articulation.set_joint_position_targets(state_vector(state))
        advance(steps)

    def teleport(state):
        vector = state_vector(state)
        articulation.set_joint_positions(vector)
        articulation.set_joint_position_targets(vector)
        advance(40)

    def world_pose(prim_path):
        prim = stage.GetPrimAtPath(prim_path)
        if not prim:
            raise RuntimeError(f"场景里找不到 prim：{prim_path}")
        # Gf.Matrix4d 是行主序、行向量约定：平移在第 4 行，旋转块相对列向量约定是转置。
        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        transform = np.eye(4)
        transform[:3, :3] = np.array(
            [[matrix[col][row] for col in range(3)] for row in range(3)], dtype=float
        )
        transform[:3, 3] = [matrix[3][0], matrix[3][1], matrix[3][2]]
        return make_pose(transform)

    def toy_pose(name):
        return world_pose(TOY_PRIMS[name])

    def rpy_to_quat_wxyz(rpy):
        """rpy（弧度，Z-Y-X 顺序）→ Isaac 用的 wxyz 四元数。"""
        roll, pitch, yaw = rpy
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        return np.array([
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ])

    def place_toy_at_input(name, pose):
        """把目标玩具刚体摆到题面输入的位姿：位置和朝向都对齐。

        恐龙资产自带 rotateXYZ=(0,0,-90)，而题面输入记的是 rpy=0；题目要求按输入位姿
        抓取，所以每例开始时先把它转正，再让物理收敛半秒，便于后续规划与录像一致。
        """
        RigidPrim(TOY_PRIMS[name]).set_world_poses(
            positions=np.array([pose["xyz"]], dtype=float),
            orientations=np.array([rpy_to_quat_wxyz(pose["rpy"])], dtype=float),
        )
        advance(TOY_SETTLE_STEPS)
        now = toy_pose(name)
        pos_gap = np.linalg.norm(np.asarray(now["xyz"]) - np.asarray(pose["xyz"], dtype=float))
        yaw_gap = math.degrees(now["rpy"][2] - pose["rpy"][2])
        yaw_gap = (yaw_gap + 180.0) % 360.0 - 180.0
        print(
            f"玩具已摆到题面位姿（含朝向 rpy=0）：位置差 {pos_gap * 1000:.1f} mm，偏航差 {yaw_gap:+.1f}°",
            flush=True,
        )
        return now

    def distance(name, objects_pose_base):
        """objects_pose_base 是底盘系位姿，玩具也换算到底盘系再比，避免跨坐标系。"""
        objects = np.asarray(objects_pose_base["xyz"])
        toy = planner.toy_transform_in_base(base_pose, toy_pose(name))[:3, 3]
        return float(np.linalg.norm(objects - toy))

    def solve_target(target_xyz, seed_state=None, rotation=None):
        """以“伸手标称姿态”为主种子求解；官方收拢起点只作兜底。

        主种子按目标方位角摆放底座，避免解出“反向转身 + 手腕反折”的怪异姿态；
        底盘已锁定，反作用不会再把这台轮式机器人推走。
        """
        return plan_for_target(chain, limits, target_xyz, rotation=rotation, seed_state=seed_state)

    def move_towards(target, frames, name):
        start = read_state()
        for frame in range(1, frames + 1):
            ratio = planner.smoothstep(frame / frames)
            drive(planner.interpolate(start, target, ratio), PHYSICS_STEPS_PER_FRAME)

    def blend_all(target, frames):
        """全部 10 个关节一起做 smoothstep 插值（含头部与手指，不只是链上 7 个）。"""
        start = read_state()
        for frame in range(1, frames + 1):
            ratio = planner.smoothstep(frame / frames)
            command = {
                key: start[key] + (target[key] - start[key]) * ratio
                for key in planner.JOINTS
            }
            drive(command, PHYSICS_STEPS_PER_FRAME)

    def align_positions(target, frames):
        """把实际关节位置分帧硬插到目标，消除物理跟随静差。

        最后一段是“保持目标等物理收敛”，但升降柱要托着手臂、PD 目标角与实际角
        天然有静差（实测可达 0.069 弧度）。若直接 teleport 到初值，这一差值会在
        单帧内被拉平，视频里就是一次瞬移。这里用 set_joint_positions 每帧只走
        残差的 smoothstep 份额，画面连续；最后一帧实际位置已等于目标，teleport
        不再产生位移。
        """
        start = read_state()
        for frame in range(1, frames + 1):
            ratio = planner.smoothstep(frame / frames)
            command = {
                key: start[key] + (target[key] - start[key]) * ratio
                for key in planner.JOINTS
            }
            vector = state_vector(command)
            articulation.set_joint_positions(vector)
            articulation.set_joint_position_targets(vector)
            advance(PHYSICS_STEPS_PER_FRAME)

    def retract_and_transfer(next_initial):
        """上一例接触后回撤到转运高度，再平滑插到下一例初值，替代瞬移。

        原来的 teleport 会让机械臂在第二、三例开头突然跳变；这里改成连续动作：
        先原地松指（避免把玩具带起来），再只升降抬到 0.30 米（不做横移，
        避免刮到桌面上其他玩具），然后在 0.30 米高度把姿态插到下一例初值，
        最后单独把升降降到位。姿态与升降分开插值是有意的：一起插值会在末段
        一边下降一边横移，实测会扫到旁边的玩具，日志里出现“意外接触”。

        松指前先带着玩具微抬 2 厘米：指尖沿玩具侧面向上脱开，比原地横向张开
        更不容易把玩具推走——实测原地张开会把魔方推偏，下一例下降时右指会擦到它。
        """
        before = {toy: world_pose(prim)["xyz"] for toy, prim in TOY_PRIMS.items()}

        micro = read_state()
        micro["lift_joint"] = min(TRANSFER_HEIGHT, micro["lift_joint"] + 0.02)
        blend_all(micro, TRANSFER_MICRO_LIFT_FRAMES)

        opened = read_state()
        opened["arm_finger_joint1"] = FINGER_OPEN
        opened["arm_finger_joint2"] = FINGER_OPEN
        blend_all(opened, TRANSFER_FINGER_FRAMES)
        blend_all(opened, 10)  # 松指后停一拍，等玩具落稳

        lifted = read_state()
        lifted["lift_joint"] = TRANSFER_HEIGHT
        blend_all(lifted, TRANSFER_LIFT_FRAMES)

        target = planner.full_state(next_initial)
        raised = dict(target)
        raised["lift_joint"] = TRANSFER_HEIGHT
        blend_all(raised, TRANSFER_BLEND_FRAMES)
        blend_all(target, TRANSFER_LIFT_DOWN_FRAMES)

        # 物理跟随有滞后：最后一帧命令发出时关节还差一点，若立即 teleport，
        # 视频里就是一个孤立尖峰。先保持目标命令等物理尽量收敛……
        residual = 0.0
        for _ in range(TRANSFER_SETTLE_FRAMES):
            drive(target, PHYSICS_STEPS_PER_FRAME)
            actual = read_state()
            residual = max(abs(actual[key] - target[key]) for key in planner.JOINTS)
            if residual <= 0.002:
                break
        worst = max(planner.JOINTS, key=lambda key: abs(read_state()[key] - target[key]))
        print(f"  过渡驱动残差 {residual:.4f}（最大关节 {worst}）", flush=True)
        # ……剩下的静差（重力压着升降柱与手臂，PD 到不了目标）用分帧硬插补齐：
        # 每帧只走残差的 1/N，视觉连续，最后 teleport 就变成无位移校正。
        align_positions(target, TRANSFER_ALIGN_FRAMES)
        residual = max(abs(read_state()[key] - target[key]) for key in planner.JOINTS)
        print(f"  过渡对齐残差 {residual:.4f}", flush=True)

        for toy, prim in TOY_PRIMS.items():
            after = world_pose(prim)["xyz"]
            shift = float(np.linalg.norm(np.array(after) - np.array(before[toy])))
            print(f"  过渡后 {toy} 位移 {shift * 1000:.1f} mm", flush=True)

    def settle_to(target, target_xyz, tolerance=0.02, max_frames=240):
        """保持目标直到夹爪真的到位。

        从官方停车位换到舒展姿态要转 2.6 弧度左右，关节速度受限，约需 3~5 秒；
        固定帧数会在机械臂还在半路时就开始下降，导致够不到玩具。
        """
        error = float("inf")
        for _ in range(max_frames):
            drive(target, PHYSICS_STEPS_PER_FRAME)
            objects = planner.pose_in_parent(planner.fk_objects_link(chain, read_state()))
            error = float(np.linalg.norm(np.asarray(objects["xyz"]) - np.asarray(target_xyz)))
            if error <= tolerance:
                break
        return error

    def finger_effort():
        """两指关节力矩绝对值的最大值，作为“真的碰到”的物理信号。"""
        values = np.asarray(articulation.get_measured_joint_efforts(), dtype=float).reshape(-1)
        return max(
            abs(float(values[index]))
            for index, joint in enumerate(dof_names)
            if "finger" in joint
        )

    def hold(steps=40):
        """把目标锁在当前关节位置，给物理时间稳定，避免继续压着玩具。"""
        drive(read_state(), steps)

    def close_fingers(steps=60, gap=0.0):
        """合拢两指直到夹住玩具：力矩超过接触阈值就停，返回过程中的峰值力矩。

        夹爪张着探到玩具上方时，指关节几乎不受力（实测 0.01~0.02 N·m），
        距离再近也不能证明碰到；只有合拢手指被玩具挡住，力矩才会升上来。
        gap 是闭合终点保留的指关节角（弧度），默认 0；留缝会削弱夹持力矩。
        """
        start = read_state()
        peak = 0.0
        for frame in range(1, steps + 1):
            ratio = planner.smoothstep(frame / steps)
            command = read_state()
            for key in ("arm_finger_joint1", "arm_finger_joint2"):
                command[key] = start[key] * (1.0 - ratio) + gap * ratio
            drive(command, PHYSICS_STEPS_PER_FRAME)
            peak = max(peak, finger_effort())
            if frame % 10 == 0:
                print(f"  合拢 {frame:03d}/{steps} 指力矩 {peak:.3f} N·m", flush=True)
            if peak > EFFORT_CONTACT:
                print(f"  合拢到第 {frame} 步夹住玩具：力矩 {peak:.3f} N·m", flush=True)
                break
        hold(20)
        return max(peak, finger_effort())

    def descend_until_contact(target, target_xyz, name, frames=120, rise_limit=0.003):
        """一次干净下降：距离进入 6 cm 后逐帧监测，触底回升即判定接触并停止。

        到达中心仍无接触时，才按 1 cm 步长继续下探。全程不反复追着被推走的玩具跑。
        返回（最优距离, 是否检测到接触, 下降过程中的峰值指力矩）。
        """
        start = read_state()
        best = float("inf")
        contact = False
        peak_effort = 0.0
        for frame in range(1, frames + 1):
            ratio = planner.smoothstep(frame / frames)
            drive(planner.interpolate(start, target, ratio), PHYSICS_STEPS_PER_FRAME)
            near = best < 0.06
            if not near and frame % 4:
                continue
            current = distance(name, planner.pose_in_parent(planner.fk_objects_link(chain, read_state())))
            effort_now = finger_effort()
            peak_effort = max(peak_effort, effort_now)
            if current < best:
                best = current
            elif best < 0.06 and current > best + rise_limit:
                # 只有已经贴近玩具（6 cm 内）时的回升才算“压到表面”。下降初段手腕转动
                # 会让 objects_link 先向外摆几毫米，远处回升不是接触，不能当判据。
                print(f"  step {frame:03d} 距离 {current:.4f} m（触底回升，判定接触）", flush=True)
                contact = True
                break
            # 手指已经压住玩具（力矩升高）时也停止，别继续往下压把玩具推走。
            if current < 0.06 and effort_now > EFFORT_CONTACT:
                print(
                    f"  step {frame:03d} 距离 {current:.4f} m，指力矩 {effort_now:.3f} N·m（判定接触）",
                    flush=True,
                )
                contact = True
                break
            if frame % 12 == 0:
                print(f"  step {frame:03d} 距离 {current:.4f} m", flush=True)

        if not contact and best > 0.06:
            # 关节有速度上限，插值结束时可能还没真正走到抓取点。保持目标再等一段，
            # 直到贴近玩具或触底，避免“目标已下发、机械臂还在半路”被误判为失败。
            print("  目标已下发，保持等待关节到位", flush=True)
            for frame in range(1, 61):
                drive(target, PHYSICS_STEPS_PER_FRAME)
                current = distance(name, planner.pose_in_parent(planner.fk_objects_link(chain, read_state())))
                if current < best:
                    best = current
                elif best < 0.06 and current > best + rise_limit:
                    print(f"  hold {frame:03d} 距离 {current:.4f} m（触底回升，判定接触）", flush=True)
                    contact = True
                    break
                effort_now = finger_effort()
                peak_effort = max(peak_effort, effort_now)
                if current < 0.06 and effort_now > EFFORT_CONTACT:
                    print(
                        f"  hold {frame:03d} 距离 {current:.4f} m，指力矩 {effort_now:.3f} N·m（判定接触）",
                        flush=True,
                    )
                    contact = True
                    break
                if frame % 15 == 0:
                    print(f"  hold {frame:03d} 距离 {current:.4f} m", flush=True)

        if not contact and best > 0.02:
            print(f"  到达中心仍未接触（指力矩 {finger_effort():.3f} N·m），继续下探", flush=True)
            for step in range(1, DEEPEN_STEPS + 1):
                deeper = np.asarray(target_xyz, dtype=float) - np.array([0.0, 0.0, DEEPEN_DEPTH * step])
                solution, error, _ = solve_target(
                    deeper, read_state(), planner.approach_rotation(deeper)
                )
                if error > 0.005:
                    print(f"  下探 {step} cm 不可达（残差 {error:.4f} m），停止", flush=True)
                    break
                segment_start = read_state()
                for frame in range(1, 7):
                    drive(
                        planner.interpolate(segment_start, solution, planner.smoothstep(frame / 6)),
                        PHYSICS_STEPS_PER_FRAME,
                    )
                current = distance(name, planner.pose_in_parent(planner.fk_objects_link(chain, read_state())))
                effort_now = finger_effort()
                peak_effort = max(peak_effort, effort_now)
                print(f"  下探 {step} cm：距离 {current:.4f} m，指力矩 {effort_now:.3f} N·m", flush=True)
                if current < best:
                    best = current
                elif current > best + 0.01:
                    contact = True
                    break
        hold()
        contact = contact or finger_effort() > EFFORT_CONTACT
        return best, contact, max(peak_effort, finger_effort())

    base_world = world_pose(BASE_PRIM)
    base_pose = {"xyz": base_world["xyz"], "rpy": base_world["rpy"]}
    results = []

    for index, record in enumerate(records):
        name = record["name"]
        case_id = record["id"]
        print(f"\n=== {case_id} ({name}) ===", flush=True)

        initial = planner.full_state(record["joint_state"])
        if index:
            contact_monitor.pause()  # 过渡动作不计入本例接触证据
            print(
                f"过渡：松指后抬到 {TRANSFER_HEIGHT:.2f} 米，再平滑插值到本例初值",
                flush=True,
            )
            retract_and_transfer(record["joint_state"])
        teleport(initial)
        place_toy_at_input(name, record["scene_object_in_world"])
        contact_monitor.begin(TOY_PRIMS[name])
        actual = read_state()
        initial_error = max(abs(actual[key] - initial[key]) for key in planner.JOINTS)
        print(f"初值读回最大误差 {initial_error:.6f} rad/m", flush=True)

        base_from_input = planner.toy_transform_in_base(base_pose, record["scene_object_in_world"])
        toy_now = toy_pose(name)
        base_from_scene = planner.toy_transform_in_base(base_pose, toy_now)
        delta = np.linalg.norm(base_from_input[:3, 3] - base_from_scene[:3, 3])
        print(
            f"玩具在底盘系：输入 {np.round(base_from_input[:3, 3], 4).tolist()}，"
            f"场景 {np.round(base_from_scene[:3, 3], 4).tolist()}，差 {delta:.4f} m",
            flush=True,
        )
        # 玩具包围盒：objects_link 是掌部参考点，距法兰 0.12236 m，不能用固定 3 cm
        # 去衡量“是否碰到”；同时给出玩具自身尺寸，便于解释最终距离的几何下限。
        toy_bounds = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(), [UsdGeom.Tokens.default_]
        ).ComputeWorldBound(stage.GetPrimAtPath(TOY_PRIMS[name])).ComputeAlignedRange()
        toy_size = np.asarray(toy_bounds.GetSize(), dtype=float)
        toy_radius = float(np.linalg.norm(toy_size) / 2)
        print(
            f"玩具包围盒尺寸 {np.round(toy_size, 4).tolist()} m，"
            f"半对角线 {toy_radius:.4f} m",
            flush=True,
        )

        target_xyz = base_from_scene[:3, 3].copy()
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
        pregrasp, pre_error, _ = solve_target(pre_xyz, initial, grasp_rotation)
        grasp, grasp_error, grasp_ori = solve_target(grasp_xyz, initial, grasp_rotation)
        if open_grasp:
            for pose in (pregrasp, grasp):
                pose["arm_finger_joint1"] = FINGER_OPEN
                pose["arm_finger_joint2"] = FINGER_OPEN
        mode_name = "宽抓" if wide_grasp else ("侧抱" if side_grasp else ("正面抓" if front_grasp else "斜抓"))
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
        transit, transit_error, _ = solve_target(
            transit_xyz, initial, planner.top_down_rotation(transit_xyz)
        )
        if open_grasp:
            transit["arm_finger_joint1"] = FINGER_OPEN
            transit["arm_finger_joint2"] = FINGER_OPEN
        print(f"  转运姿态（高 0.30 米）解残差 {transit_error:.4f} m", flush=True)
        for label, target, frames in (
            ("抬升并转向玩具", transit, 100),
            ("下降到预抓取", pregrasp, 90),
        ):
            print(f"-- {label}", flush=True)
            move_towards(target, frames, name)
            below = planner.link_origins_below_floor(chain, read_state())
            if below:
                print("  警告：连杆原点低于地面", below, flush=True)
        print("-- 等待预抓取到位", flush=True)
        settle_error = settle_to(pregrasp, pre_xyz)
        print(f"  预抓取到位误差 {settle_error:.4f} m", flush=True)

        print("-- 下降到玩具", flush=True)
        toy_before = planner.toy_transform_in_base(base_pose, toy_pose(name))[:3, 3].copy()
        if open_grasp:
            # 宽抓/侧抱：不再“碰到就停”，而是降到两指包住玩具的高度，再合拢夹住。
            move_towards(grasp, 120, name)
            print("-- 等待宽抓到位", flush=True)
            wide_error = settle_to(grasp, grasp_xyz)
            print(f"  宽抓到位误差 {wide_error:.4f} m", flush=True)
            advance(20)
            best_distance = distance(name, planner.pose_in_parent(planner.fk_objects_link(chain, read_state())))
            rebound = False
            descent_effort = finger_effort()
        else:
            best_distance, rebound, descent_effort = descend_until_contact(grasp, target_xyz, name)
        # 接触后先让物理稳定，再读最终关节；否则拿到的可能是插值过程中的瞬时状态。
        advance(20)
        before_close = distance(name, planner.pose_in_parent(planner.fk_objects_link(chain, read_state())))
        if rebound and not open_grasp:
            # 已经停在玩具表面：再合拢只会把玩具推走，跳过。
            # 但下降途中压到玩具时测到的指力矩要保留，否则报告会变成“接触但几乎无力”。
            print("-- 已停在玩具表面，跳过合拢", flush=True)
            peak_effort = descent_effort
        else:
            print("-- 合拢手指", flush=True)
            peak_effort = close_fingers(steps=90 if open_grasp else 60, gap=args.finger_gap)
        current_distance = distance(name, planner.pose_in_parent(planner.fk_objects_link(chain, read_state())))
        effort = max(peak_effort, finger_effort())
        toy_shift = float(np.linalg.norm(
            planner.toy_transform_in_base(base_pose, toy_pose(name))[:3, 3] - toy_before
        ))
        # 触碰的物理证据：PhysX 上报的真实碰撞对，或指关节被夹住、下降触底停顿、玩具被推离原位。
        # 指力矩测不到“自上而下压”，因为力沿指连杆方向，不绕指关节轴。
        sensor_event = contact_monitor.target
        contact = effort > EFFORT_CONTACT or rebound or toy_shift > TOY_DISPLACEMENT
        if sensor_event is not None:
            print(
                f"  PhysX 接触对 {sensor_event['actors']} "
                f"分离 {sensor_event['separation_m']:.6f} m，"
                f"冲量 {sensor_event['impulse_ns']:.6f} N·s",
                flush=True,
            )
        else:
            print(f"  PhysX 接触 未上报（本测例共 {contact_monitor.events} 个有效接触点）", flush=True)
        if contact_monitor.unexpected:
            print(f"  意外接触 {contact_monitor.unexpected[0]['actors']}", flush=True)
        print(
            f"接触距离 {current_distance:.4f} m（合拢前 {before_close:.4f} m，最优 {best_distance:.4f} m，"
            f"触底回升 {'是' if rebound else '否'}，峰值指力矩 {effort:.3f} N·m，"
            f"玩具位移 {toy_shift * 1000:.1f} mm，接触 {'有' if contact else '无'}）",
            flush=True,
        )
        efforts = np.asarray(articulation.get_measured_joint_efforts(), dtype=float).reshape(-1)
        finger_efforts = {
            joint: round(float(efforts[index]), 4)
            for index, joint in enumerate(dof_names)
            if "finger" in joint
        }
        print("  指关节力矩", finger_efforts, flush=True)

        final_state = read_state()
        final_errors = {
            key: (final_state[key], limits[key])
            for key in planner.JOINTS
            if key in limits and not (limits[key][0] - 1e-4 <= final_state[key] <= limits[key][1] + 1e-4)
        }
        if final_errors:
            raise RuntimeError(f"{case_id} 最终关节越限：{final_errors}")

        objects_in_base = planner.fk_objects_link(chain, final_state)
        objects_pose = planner.pose_in_parent(objects_in_base)
        usd_objects_in_base = planner.toy_transform_in_base(base_pose, world_pose(OBJECTS_PRIM))
        usd_delta = float(
            np.linalg.norm(usd_objects_in_base[:3, 3] - np.asarray(objects_pose["xyz"]))
        )
        # 自检：底盘有没有被推走，用来解释 USD/FK 偏差。
        base_now = world_pose(BASE_PRIM)
        base_shift = float(np.linalg.norm(np.asarray(base_now["xyz"]) - np.asarray(base_pose["xyz"])))
        print(f"  底盘位移 {base_shift:.4f} m", flush=True)
        # 题面 2.4 节规定 scene_object_in_world 就是玩具的世界位姿，所以答案主字段按输入算（恐龙 rpy=0）。
        # 场景里恐龙刚体自带 rotateXYZ=(0,0,-90)，仿真实际位姿与输入口径差约 90°，单独存为附加字段。
        toy_in_base = planner.toy_transform_in_base(base_pose, toy_pose(name))
        toy_in_base_from_input = planner.toy_transform_in_base(
            record["base_footprint_in_world"], record["scene_object_in_world"]
        )
        toy_in_objects = planner.toy_pose_in_objects(objects_in_base, toy_in_base_from_input)
        toy_in_objects_sim = planner.toy_pose_in_objects(objects_in_base, toy_in_base)
        yaw_gap = math.degrees(
            math.atan2(toy_in_base[1, 0], toy_in_base[0, 0])
            - math.atan2(toy_in_base_from_input[1, 0], toy_in_base_from_input[0, 0])
        )
        yaw_gap = (yaw_gap + 180.0) % 360.0 - 180.0
        input_gap = float(np.linalg.norm(toy_in_base[:3, 3] - toy_in_base_from_input[:3, 3]))
        print(
            f"  玩具位姿：主字段按题面输入（rpy=0）计算，仿真实测另存；"
            f"两者相差 {input_gap:.4f} m / {yaw_gap:+.1f}°（资产自带旋转 + 触碰推动所致）",
            flush=True,
        )
        touch_distance = float(np.linalg.norm(toy_in_base[:3, 3] - objects_in_base[:3, 3]))
        # 触碰判据：PhysX 接触报告或物理证据（回升/力矩/位移）优先；距离只作几何兜底——
        # 玩具被撞飞时同样会上报接触，但它已不在夹爪工作范围，不能算“碰到”。
        distance_limit = max(TOUCH_DISTANCE, toy_radius)
        passed = decide_touch(contact, sensor_event) and touch_distance <= distance_limit
        print(
            f"完成：最终距离 {touch_distance:.4f} m（几何参考上限 {distance_limit:.4f} m），"
            f"指力矩 {effort:.3f} N·m，USD/FK 偏差 {usd_delta:.5f} m，{'PASS' if passed else 'FAIL'}",
            flush=True,
        )

        result = {
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
            "unexpected_contact": contact_monitor.unexpected[0] if contact_monitor.unexpected else None,
            "finger_effort_nm": round(effort, 4),
            "toy_displacement_m": round(toy_shift, 6),
            "initial_state_error_max": round(initial_error, 6),
        }
        results.append(result)

    if args.only and args.output is None:
        print(f"\n[SKIP] 只跑了 {len(results)} 个测例（--only），未写答案；需要写请显式指定 --output", flush=True)
    else:
        output = Path(args.output or (ENTRY_ROOT / "answers/q3_demo_output.jsonl"))
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("w", encoding="utf-8") as handle:
            for item in results:
                handle.write(json.dumps(item, ensure_ascii=False) + "\n")
        print(f"\n已写入 {len(results)} 行 -> {output}", flush=True)

    if writer is not None:
        writer.release()
        print(f"视频已写出 -> {args.video}（{frame_count} 个物理步）", flush=True)

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

    contact_monitor.stop()

    # 图形模式下保持窗口一段时间，方便看清最后一个玩具的最终姿态；
    # 无窗口模式立即退出，避免影响批量运行。
    hold = args.hold_sec if args.hold_sec is not None else (0 if args.headless else 90)
    if hold > 0:
        print(f"\n窗口保持 {hold:.0f} 秒（--hold-sec 可调，0 为立即退出）", flush=True)
        for _ in range(int(hold * 30)):
            world.step(render=True)

    simulation_app.close()


if __name__ == "__main__":
    main()
