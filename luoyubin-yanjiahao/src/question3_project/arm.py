#!/usr/bin/env python3
"""题三机械臂动作原语：读状态、插值下发、过渡、下降与合拢。

只负责“发什么关节目标、发多少帧”，物理与底盘锁定由 Sim 处理。
"""
import numpy as np

import config
import planner
from planning import plan_for_target


class Arm:
    """持有一条关节链与底盘位姿，把关节目标变成连续动作。"""

    def __init__(self, sim, chain, limits):
        self.sim = sim
        self.chain = chain
        self.limits = limits
        base_world = sim.world_pose(config.BASE_PRIM)
        self.base_pose = {"xyz": base_world["xyz"], "rpy": base_world["rpy"]}

    def read_state(self):
        return self.sim.read_state()

    def drive(self, state, steps):
        self.sim.drive(state, steps)

    def solve_target(self, target_xyz, seed_state=None, rotation=None):
        """以“伸手标称姿态”为主种子求解；官方收拢起点只作兜底。

        主种子按目标方位角摆放底座，避免解出“反向转身 + 手腕反折”的怪异姿态；
        底盘已锁定，反作用不会再把这台轮式机器人推走。
        """
        return plan_for_target(
            self.chain, self.limits, target_xyz, rotation=rotation, seed_state=seed_state
        )

    def distance(self, name, objects_pose_base):
        """objects_pose_base 是底盘系位姿，玩具也换算到底盘系再比，避免跨坐标系。"""
        objects = np.asarray(objects_pose_base["xyz"])
        toy = planner.toy_transform_in_base(self.base_pose, self.sim.toy_pose(name))[:3, 3]
        return float(np.linalg.norm(objects - toy))

    def objects_pose(self):
        return planner.pose_in_parent(planner.fk_objects_link(self.chain, self.read_state()))

    def move_towards(self, target, frames, name):
        start = self.read_state()
        for frame in range(1, frames + 1):
            ratio = planner.smoothstep(frame / frames)
            self.drive(planner.interpolate(start, target, ratio), config.PHYSICS_STEPS_PER_FRAME)

    def blend_all(self, target, frames):
        """全部 10 个关节一起做 smoothstep 插值（含头部与手指，不只是链上 7 个）。"""
        start = self.read_state()
        for frame in range(1, frames + 1):
            ratio = planner.smoothstep(frame / frames)
            command = {
                key: start[key] + (target[key] - start[key]) * ratio
                for key in planner.JOINTS
            }
            self.drive(command, config.PHYSICS_STEPS_PER_FRAME)

    def align_positions(self, target, frames):
        """把实际关节位置分帧硬插到目标，消除物理跟随静差。

        最后一段是“保持目标等物理收敛”，但升降柱要托着手臂、PD 目标角与实际角
        天然有静差（实测可达 0.069 弧度）。若直接 teleport 到初值，这一差值会在
        单帧内被拉平，视频里就是一次瞬移。这里用 set_joint_positions 每帧只走
        残差的 smoothstep 份额，画面连续；最后一帧实际位置已等于目标，teleport
        不再产生位移。
        """
        start = self.read_state()
        for frame in range(1, frames + 1):
            ratio = planner.smoothstep(frame / frames)
            command = {
                key: start[key] + (target[key] - start[key]) * ratio
                for key in planner.JOINTS
            }
            vector = self.sim.state_vector(command)
            self.sim.articulation.set_joint_positions(vector)
            self.sim.articulation.set_joint_position_targets(vector)
            self.sim.advance(config.PHYSICS_STEPS_PER_FRAME)

    def retract_and_transfer(self, next_initial):
        """上一例接触后回撤到转运高度，再平滑插到下一例初值，替代瞬移。

        原来的 teleport 会让机械臂在第二、三例开头突然跳变；这里改成连续动作：
        先原地松指（避免把玩具带起来），再只升降抬到 0.30 米（不做横移，
        避免刮到桌面上其他玩具），然后在 0.30 米高度把姿态插到下一例初值，
        最后单独把升降降到位。姿态与升降分开插值是有意的：一起插值会在末段
        一边下降一边横移，实测会扫到旁边的玩具，日志里出现“意外接触”。

        松指前先带着玩具微抬 2 厘米：指尖沿玩具侧面向上脱开，比原地横向张开
        更不容易把玩具推走——实测原地张开会把魔方推偏，下一例下降时右指会擦到它。
        """
        before = {toy: self.sim.world_pose(prim)["xyz"] for toy, prim in config.TOY_PRIMS.items()}

        micro = self.read_state()
        micro["lift_joint"] = min(config.TRANSFER_HEIGHT, micro["lift_joint"] + 0.02)
        self.blend_all(micro, config.TRANSFER_MICRO_LIFT_FRAMES)

        opened = self.read_state()
        opened["arm_finger_joint1"] = config.FINGER_OPEN
        opened["arm_finger_joint2"] = config.FINGER_OPEN
        self.blend_all(opened, config.TRANSFER_FINGER_FRAMES)
        self.blend_all(opened, 10)  # 松指后停一拍，等玩具落稳

        lifted = self.read_state()
        lifted["lift_joint"] = config.TRANSFER_HEIGHT
        self.blend_all(lifted, config.TRANSFER_LIFT_FRAMES)

        target = planner.full_state(next_initial)
        raised = dict(target)
        raised["lift_joint"] = config.TRANSFER_HEIGHT
        self.blend_all(raised, config.TRANSFER_BLEND_FRAMES)
        self.blend_all(target, config.TRANSFER_LIFT_DOWN_FRAMES)

        # 物理跟随有滞后：最后一帧命令发出时关节还差一点，若立即 teleport，
        # 视频里就是一个孤立尖峰。先保持目标命令等物理尽量收敛……
        residual = 0.0
        for _ in range(config.TRANSFER_SETTLE_FRAMES):
            self.drive(target, config.PHYSICS_STEPS_PER_FRAME)
            actual = self.read_state()
            residual = max(abs(actual[key] - target[key]) for key in planner.JOINTS)
            if residual <= 0.002:
                break
        worst = max(planner.JOINTS, key=lambda key: abs(self.read_state()[key] - target[key]))
        print(f"  过渡驱动残差 {residual:.4f}（最大关节 {worst}）", flush=True)
        # ……剩下的静差（重力压着升降柱与手臂，PD 到不了目标）用分帧硬插补齐：
        # 每帧只走残差的 1/N，视觉连续，最后 teleport 就变成无位移校正。
        self.align_positions(target, config.TRANSFER_ALIGN_FRAMES)
        residual = max(abs(self.read_state()[key] - target[key]) for key in planner.JOINTS)
        print(f"  过渡对齐残差 {residual:.4f}", flush=True)

        for toy, prim in config.TOY_PRIMS.items():
            after = self.sim.world_pose(prim)["xyz"]
            shift = float(np.linalg.norm(np.array(after) - np.array(before[toy])))
            print(f"  过渡后 {toy} 位移 {shift * 1000:.1f} mm", flush=True)

    def settle_to(self, target, target_xyz, tolerance=0.02, max_frames=240):
        """保持目标直到夹爪真的到位。

        从官方停车位换到舒展姿态要转 2.6 弧度左右，关节速度受限，约需 3~5 秒；
        固定帧数会在机械臂还在半路时就开始下降，导致够不到玩具。
        """
        error = float("inf")
        for _ in range(max_frames):
            self.drive(target, config.PHYSICS_STEPS_PER_FRAME)
            objects = planner.pose_in_parent(planner.fk_objects_link(self.chain, self.read_state()))
            error = float(np.linalg.norm(np.asarray(objects["xyz"]) - np.asarray(target_xyz)))
            if error <= tolerance:
                break
        return error

    def finger_effort(self):
        """两指关节力矩绝对值的最大值，作为“真的碰到”的物理信号。"""
        values = np.asarray(self.sim.articulation.get_measured_joint_efforts(), dtype=float).reshape(-1)
        return max(
            abs(float(values[index]))
            for index, joint in enumerate(self.sim.dof_names)
            if "finger" in joint
        )

    def hold(self, steps=40):
        """把目标锁在当前关节位置，给物理时间稳定，避免继续压着玩具。"""
        self.drive(self.read_state(), steps)

    def close_fingers(self, steps=60, gap=0.0):
        """合拢两指直到夹住玩具：力矩超过接触阈值就停，返回过程中的峰值力矩。

        夹爪张着探到玩具上方时，指关节几乎不受力（实测 0.01~0.02 N·m），
        距离再近也不能证明碰到；只有合拢手指被玩具挡住，力矩才会升上来。
        gap 是闭合终点保留的指关节角（弧度），默认 0；留缝会削弱夹持力矩。
        """
        start = self.read_state()
        peak = 0.0
        for frame in range(1, steps + 1):
            ratio = planner.smoothstep(frame / steps)
            command = self.read_state()
            for key in ("arm_finger_joint1", "arm_finger_joint2"):
                command[key] = start[key] * (1.0 - ratio) + gap * ratio
            self.drive(command, config.PHYSICS_STEPS_PER_FRAME)
            peak = max(peak, self.finger_effort())
            if frame % 10 == 0:
                print(f"  合拢 {frame:03d}/{steps} 指力矩 {peak:.3f} N·m", flush=True)
            if peak > config.EFFORT_CONTACT:
                print(f"  合拢到第 {frame} 步夹住玩具：力矩 {peak:.3f} N·m", flush=True)
                break
        self.hold(20)
        return max(peak, self.finger_effort())

    def descend_until_contact(self, target, target_xyz, name, frames=120, rise_limit=0.003):
        """一次干净下降：距离进入 6 cm 后逐帧监测，触底回升即判定接触并停止。

        到达中心仍无接触时，才按 1 cm 步长继续下探。全程不反复追着被推走的玩具跑。
        返回（最优距离, 是否检测到接触, 下降过程中的峰值指力矩）。
        """
        start = self.read_state()
        best = float("inf")
        contact = False
        peak_effort = 0.0
        for frame in range(1, frames + 1):
            ratio = planner.smoothstep(frame / frames)
            self.drive(planner.interpolate(start, target, ratio), config.PHYSICS_STEPS_PER_FRAME)
            near = best < 0.06
            if not near and frame % 4:
                continue
            current = self.distance(name, self.objects_pose())
            effort_now = self.finger_effort()
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
            if current < 0.06 and effort_now > config.EFFORT_CONTACT:
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
                self.drive(target, config.PHYSICS_STEPS_PER_FRAME)
                current = self.distance(name, self.objects_pose())
                if current < best:
                    best = current
                elif best < 0.06 and current > best + rise_limit:
                    print(f"  hold {frame:03d} 距离 {current:.4f} m（触底回升，判定接触）", flush=True)
                    contact = True
                    break
                effort_now = self.finger_effort()
                peak_effort = max(peak_effort, effort_now)
                if current < 0.06 and effort_now > config.EFFORT_CONTACT:
                    print(
                        f"  hold {frame:03d} 距离 {current:.4f} m，指力矩 {effort_now:.3f} N·m（判定接触）",
                        flush=True,
                    )
                    contact = True
                    break
                if frame % 15 == 0:
                    print(f"  hold {frame:03d} 距离 {current:.4f} m", flush=True)

        if not contact and best > 0.02:
            print(f"  到达中心仍未接触（指力矩 {self.finger_effort():.3f} N·m），继续下探", flush=True)
            for step in range(1, config.DEEPEN_STEPS + 1):
                deeper = np.asarray(target_xyz, dtype=float) - np.array(
                    [0.0, 0.0, config.DEEPEN_DEPTH * step]
                )
                solution, error, _ = self.solve_target(
                    deeper, self.read_state(), planner.approach_rotation(deeper)
                )
                if error > 0.005:
                    print(f"  下探 {step} cm 不可达（残差 {error:.4f} m），停止", flush=True)
                    break
                segment_start = self.read_state()
                for frame in range(1, 7):
                    self.drive(
                        planner.interpolate(segment_start, solution, planner.smoothstep(frame / 6)),
                        config.PHYSICS_STEPS_PER_FRAME,
                    )
                current = self.distance(name, self.objects_pose())
                effort_now = self.finger_effort()
                peak_effort = max(peak_effort, effort_now)
                print(f"  下探 {step} cm：距离 {current:.4f} m，指力矩 {effort_now:.3f} N·m", flush=True)
                if current < best:
                    best = current
                elif current > best + 0.01:
                    contact = True
                    break
        self.hold()
        contact = contact or self.finger_effort() > config.EFFORT_CONTACT
        return best, contact, max(peak_effort, self.finger_effort())
