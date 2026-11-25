#!/usr/bin/env python3
"""题三 Isaac Sim 运行时：启动应用、打开场景、推进物理、锁底盘、录视频。

SimulationApp 必须先于其他 isaac 模块创建，所以 Isaac 相关导入都放在
`Sim.__init__` 内部，模块顶层可被无 Isaac 的环境安全导入。
"""
import math
import sys
from pathlib import Path

import numpy as np

PROJECT_DIR = Path(__file__).resolve().parent
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

import config
from contact import ContactMonitor
from input_io import make_pose


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


class Sim:
    """一次仿真的全部运行时对象；关节控制语义留在 Arm 里。"""

    def __init__(self, scene_path, headless=False, video=None):
        from isaacsim import SimulationApp

        self.headless = headless
        self.video_path = video
        self.simulation_app = SimulationApp({"headless": headless, "width": 1280, "height": 720})
        print("ISAAC_OK", flush=True)

        import omni.usd
        from isaacsim.core.utils.extensions import enable_extension

        enable_extension("isaacsim.ros2.bridge")
        for _ in range(5):
            self.simulation_app.update()

        context = omni.usd.get_context()
        context.open_stage(str(scene_path))
        stage = None
        for _ in range(1200):
            self.simulation_app.update()
            stage = context.get_stage()
            if stage is not None and stage.GetDefaultPrim() and stage.GetPrimAtPath(config.ROBOT_ROOT):
                break
        if stage is None or not stage.GetPrimAtPath(config.ROBOT_ROOT):
            raise RuntimeError("场景加载失败或缺少 robot_root")
        self.stage = stage
        print("STAGE_OK", stage.GetDefaultPrim().GetPath(), flush=True)
        self.contact_monitor = ContactMonitor(stage, config.ROBOT_ROOT, config.TOY_PRIMS.values())
        print("CONTACT_REPORT_ON", len(config.TOY_PRIMS), "个玩具", flush=True)

        if not headless:
            # 图形窗口里把视口相机对准机器人工作区，否则默认视角看不到三个玩具。
            from isaacsim.core.utils.viewports import set_camera_view
            from omni.kit.viewport.utility import get_active_viewport

            set_camera_view(eye=config.VIDEO_EYE, target=config.VIDEO_TARGET)
            active = get_active_viewport()
            print("VIEWPORT", None if active is None else active.resolution, flush=True)

        from isaacsim.core.api import World

        self.world = World()
        self.world.play()
        for _ in range(120):
            self.world.step(render=not headless)
        print("PLAY_OK", flush=True)

        # 视频：独立相机 + RGB 标注器。world.render() 只渲染、不推进物理，能同步取到画面。
        self.writer = None
        self.frame_count = 0
        if video:
            import cv2
            import omni.replicator.core as rep

            self.cv2 = cv2
            camera = rep.create.camera(
                position=config.VIDEO_EYE, look_at=config.VIDEO_TARGET, name="q3_video_camera"
            )
            render_product = rep.create.render_product(camera, (config.VIDEO_WIDTH, config.VIDEO_HEIGHT))
            self.rgb = rep.AnnotatorRegistry.get_annotator("rgb")
            self.rgb.attach([render_product])
            self.writer = cv2.VideoWriter(
                str(video), cv2.VideoWriter_fourcc(*config.VIDEO_CODEC), config.VIDEO_FPS,
                (config.VIDEO_WIDTH, config.VIDEO_HEIGHT),
            )
            if not self.writer.isOpened():
                raise RuntimeError(f"无法创建视频文件：{video}")
            print("VIDEO", video, config.VIDEO_WIDTH, config.VIDEO_HEIGHT, flush=True)
        self.render_each_step = (not headless) or (self.writer is not None)

        from isaacsim.core.prims import Articulation, RigidPrim

        self.RigidPrim = RigidPrim
        self.articulation = Articulation(config.BASE_PRIM)
        self.articulation.initialize()
        self.dof_names = list(self.articulation.dof_names)
        print("DOF", self.dof_names, flush=True)

        # 底盘锁定：这台机器人是轮式自由 articulation，机械臂伸展时的反作用会把整台车推走
        # （实测 1~33 厘米），答案位姿与真实位姿就错开了。该 Isaac 版本没有 fixRootJoint，
        # 所以在每个物理步之后把根位姿写回初始值，等价于把底盘钉在原地。
        try:
            root_position, root_orientation = self.articulation.get_world_poses()
            self.root_position = np.asarray(root_position, dtype=float).copy()
            self.root_orientation = np.asarray(root_orientation, dtype=float).copy()
        except Exception as exc:  # 拿不到根位姿就退回自由底盘，并在日志里说清
            self.root_position = self.root_orientation = None
            print(f"BASE_LOCK 失败：{exc}（底盘将自由移动）", flush=True)
        if self.root_position is not None:
            print("BASE_LOCK_ON", np.round(self.root_position.reshape(-1), 4).tolist(), flush=True)

    def lock_base(self):
        if self.root_position is None:
            return
        self.articulation.set_world_poses(self.root_position, self.root_orientation)

    def advance(self, steps=1):
        """推进物理；录视频时每 VIDEO_STRIDE 步渲染一帧写进视频（不额外推进物理）。

        图形模式下每步都要渲染，否则窗口停在第一帧、看不到机械臂动作。
        """
        for _ in range(steps):
            self.world.step(render=self.render_each_step)
            self.lock_base()
            if self.writer is None:
                continue
            self.frame_count += 1
            if self.frame_count % config.VIDEO_STRIDE:
                continue
            self.world.render()
            data = self.rgb.get_data()
            if data is None or not np.asarray(data).size:
                continue
            image = np.asarray(data)[:, :, :3]
            if image.dtype != np.uint8:
                image = np.clip(image, 0, 255).astype(np.uint8)
            self.writer.write(self.cv2.cvtColor(image, self.cv2.COLOR_RGB2BGR))

    def read_state(self):
        values = np.asarray(self.articulation.get_joint_positions(), dtype=float).reshape(-1)
        return {name: float(values[i]) for i, name in enumerate(self.dof_names)}

    def state_vector(self, state):
        values = np.asarray(self.articulation.get_joint_positions(), dtype=float).reshape(-1)
        for index, name in enumerate(self.dof_names):
            if name in state:
                values[index] = float(state[name])
        return values.reshape(1, -1)

    def drive(self, state, steps):
        self.articulation.set_joint_position_targets(self.state_vector(state))
        self.advance(steps)

    def teleport(self, state):
        vector = self.state_vector(state)
        self.articulation.set_joint_positions(vector)
        self.articulation.set_joint_position_targets(vector)
        self.advance(40)

    def world_pose(self, prim_path):
        from pxr import Usd, UsdGeom

        prim = self.stage.GetPrimAtPath(prim_path)
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

    def toy_pose(self, name):
        return self.world_pose(config.TOY_PRIMS[name])

    def place_toy_at_input(self, name, pose):
        """把目标玩具刚体摆到题面输入的位姿：位置和朝向都对齐。

        恐龙资产自带 rotateXYZ=(0,0,-90)，而题面输入记的是 rpy=0；题目要求按输入位姿
        抓取，所以每例开始时先把它转正，再让物理收敛半秒，便于后续规划与录像一致。
        """
        self.RigidPrim(config.TOY_PRIMS[name]).set_world_poses(
            positions=np.array([pose["xyz"]], dtype=float),
            orientations=np.array([rpy_to_quat_wxyz(pose["rpy"])], dtype=float),
        )
        self.advance(config.TOY_SETTLE_STEPS)
        now = self.toy_pose(name)
        pos_gap = np.linalg.norm(np.asarray(now["xyz"]) - np.asarray(pose["xyz"], dtype=float))
        yaw_gap = math.degrees(now["rpy"][2] - pose["rpy"][2])
        yaw_gap = (yaw_gap + 180.0) % 360.0 - 180.0
        print(
            f"玩具已摆到题面位姿（含朝向 rpy=0）：位置差 {pos_gap * 1000:.1f} mm，偏航差 {yaw_gap:+.1f}°",
            flush=True,
        )
        return now

    def release_video(self):
        if self.writer is not None:
            self.writer.release()
            print(f"视频已写出 -> {self.video_path}（{self.frame_count} 个物理步）", flush=True)

    def close(self, hold_sec=0):
        self.contact_monitor.stop()
        # 图形模式下保持窗口一段时间，方便看清最后一个玩具的最终姿态；
        # 无窗口模式立即退出，避免影响批量运行。
        if hold_sec > 0:
            print(f"\n窗口保持 {hold_sec:.0f} 秒（--hold-sec 可调，0 为立即退出）", flush=True)
            for _ in range(int(hold_sec * 30)):
                self.world.step(render=True)
        self.simulation_app.close()
