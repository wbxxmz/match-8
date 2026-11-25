#!/usr/bin/env python3
"""题三触碰证据：判定口径与 PhysX 接触报告订阅。"""
import numpy as np

from config import CONTACT_MAX_SEPARATION, CONTACT_MIN_IMPULSE


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
