#!/usr/bin/env python3
"""题二可选可视化：在 RViz 中显示 objects_link、场景物体与两者关系。

发布两个话题：
- `/joint_states`：完整 10 关节，驱动 robot_state_publisher 更新整机姿态；
- `/q2_markers`：场景物体方块、夹爪参考球、三轴箭头、两点连线与距离文字。

默认按输入姿态静止显示；`--animate` 时在输入姿态与一个展示姿态之间往返，
用来观察正运动学随关节角变化的效果。该演示不产生答案，答案仍由 run_q2.py 生成。
"""
import argparse
import math
import time

import numpy as np

from fk import JOINTS, objects_link_transform
from run_q2 import find_urdf_root, load_chain, load_records, pose_from_transform
from transforms import invert_transform, matrix_to_quat_xyzw, rpy_to_matrix

FRAME = "base_footprint"
TOPIC = "/q2_markers"
RATE_HZ = 30.0

# 演示姿态（非官方测例）：升降抬起、手臂前伸并侧摆，夹爪张开，便于观察变换关系。
DEMO_POSE = {
    "lift_joint": 0.18,
    "pitch_joint": 0.0,
    "joint1": 0.45,
    "joint2": -0.55,
    "joint3": 0.35,
    "joint4": 0.0,
    "joint5": -0.45,
    "joint6": 0.0,
    "arm_finger_joint1": 0.35,
    "arm_finger_joint2": 0.35,
}


def smoothstep(alpha):
    return alpha * alpha * (3.0 - 2.0 * alpha)


def blend(base, target, alpha):
    return {name: base[name] + (target[name] - base[name]) * alpha for name in JOINTS}


def set_color(color, rgba):
    color.r, color.g, color.b, color.a = rgba


def add_arrow(marker, start, end, rgba, shaft=0.010, head=0.022, head_len=0.030):
    from geometry_msgs.msg import Point
    marker.type = marker.ARROW
    marker.action = marker.ADD
    marker.points = [Point(x=float(start[0]), y=float(start[1]), z=float(start[2])),
                     Point(x=float(end[0]), y=float(end[1]), z=float(end[2]))]
    marker.scale.x, marker.scale.y, marker.scale.z = shaft, head, head_len
    set_color(marker.color, rgba)


def add_text(marker, position, text, rgba, height=0.05):
    marker.type = marker.TEXT_VIEW_FACING
    marker.action = marker.ADD
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = (
        float(position[0]), float(position[1]), float(position[2]))
    marker.pose.orientation.w = 1.0
    marker.scale.z = height
    marker.text = text
    set_color(marker.color, rgba)


def build_markers(record, tip_matrix):
    """构造 MarkerArray：物体、参考点、三轴、连线与文字，全部在 base_footprint 下。"""
    from visualization_msgs.msg import Marker, MarkerArray

    array = MarkerArray()
    object_pose = record["scene_object_in_base_footprint"]
    object_xyz = np.asarray(object_pose["xyz"], dtype=float)
    object_rpy = object_pose.get("rpy", [0.0, 0.0, 0.0])
    object_quat = matrix_to_quat_xyzw(rpy_to_matrix(object_rpy))
    tip_xyz = tip_matrix[:3, 3]
    tip_quat = matrix_to_quat_xyzw(tip_matrix[:3, :3])
    distance = float(np.linalg.norm(object_xyz - tip_xyz))

    cube = Marker()
    cube.id, cube.ns = 0, "scene_object"
    cube.type = Marker.CUBE
    cube.action = Marker.ADD
    cube.pose.position.x, cube.pose.position.y, cube.pose.position.z = (float(v) for v in object_xyz)
    cube.pose.orientation.x, cube.pose.orientation.y = float(object_quat[0]), float(object_quat[1])
    cube.pose.orientation.z, cube.pose.orientation.w = float(object_quat[2]), float(object_quat[3])
    cube.scale.x = cube.scale.y = cube.scale.z = 0.09
    cube.color.r, cube.color.g, cube.color.b, cube.color.a = 1.0, 0.55, 0.10, 0.85
    array.markers.append(cube)

    sphere = Marker()
    sphere.id, sphere.ns = 1, "objects_link"
    sphere.type = Marker.SPHERE
    sphere.action = Marker.ADD
    sphere.pose.position.x, sphere.pose.position.y, sphere.pose.position.z = (float(v) for v in tip_xyz)
    sphere.pose.orientation.x, sphere.pose.orientation.y = float(tip_quat[0]), float(tip_quat[1])
    sphere.pose.orientation.z, sphere.pose.orientation.w = float(tip_quat[2]), float(tip_quat[3])
    sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.028
    sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 0.95, 0.15, 0.15, 1.0
    array.markers.append(sphere)

    axes = ((0, (0.95, 0.15, 0.15, 1.0)), (1, (0.15, 0.85, 0.15, 1.0)), (2, (0.25, 0.35, 1.0, 1.0)))
    for index, rgba in axes:
        axis_world = tip_matrix[:3, index] * 0.14
        marker = Marker()
        marker.id, marker.ns = 2 + index, "objects_link_axes"
        add_arrow(marker, tip_xyz, tip_xyz + axis_world, rgba, shaft=0.006, head=0.014, head_len=0.02)
        array.markers.append(marker)

    link = Marker()
    link.id, link.ns = 5, "tip_to_object"
    add_arrow(link, tip_xyz, object_xyz, (1.0, 0.90, 0.20, 1.0), shaft=0.007, head=0.018, head_len=0.026)
    array.markers.append(link)

    label_object = Marker()
    label_object.id, label_object.ns = 6, "labels"
    add_text(label_object, object_xyz + np.array([0.0, 0.0, 0.10]),
             "scene_object", (1.0, 0.75, 0.35, 1.0), height=0.045)
    array.markers.append(label_object)

    label_link = Marker()
    label_link.id, label_link.ns = 7, "labels"
    add_text(label_link, tip_xyz + np.array([0.0, 0.0, 0.22]),
             "objects_link", (1.0, 0.45, 0.45, 1.0), height=0.04)
    array.markers.append(label_link)

    midpoint = (tip_xyz + object_xyz) / 2.0 + np.array([0.0, 0.0, 0.05])
    label_distance = Marker()
    label_distance.id, label_distance.ns = 8, "labels"
    add_text(label_distance, midpoint, f"{distance:.3f}m", (1.0, 0.95, 0.5, 1.0), height=0.05)
    array.markers.append(label_distance)

    return MarkerArray(markers=array.markers), distance


def stamp_markers(array, stamp):
    for marker in array.markers:
        marker.header.frame_id = FRAME
        marker.header.stamp = stamp


def summary(chain, record):
    """启动时打印一次可核对的数值。"""
    tip = objects_link_transform(chain, record["joint_state"])
    tip_pose = pose_from_transform(tip)
    object_in_tip = pose_from_transform(
        invert_transform(tip) @ _object_matrix(record)
    )
    distance = float(np.linalg.norm(tip[:3, 3] - np.asarray(record["scene_object_in_base_footprint"]["xyz"], float)))
    print(f"[{record['id']}] objects_link xyz: "
          f"[{tip_pose['xyz'][0]:.6f}, {tip_pose['xyz'][1]:.6f}, {tip_pose['xyz'][2]:.6f}]", flush=True)
    print(f"[{record['id']}] scene_object_in_objects_link xyz: "
          f"[{object_in_tip['xyz'][0]:.6f}, {object_in_tip['xyz'][1]:.6f}, {object_in_tip['xyz'][2]:.6f}]", flush=True)
    print(f"[{record['id']}] distance: {distance:.6f} m", flush=True)


def _object_matrix(record):
    pose = record["scene_object_in_base_footprint"]
    matrix = np.eye(4)
    matrix[:3, :3] = rpy_to_matrix(pose.get("rpy", [0.0, 0.0, 0.0]))
    matrix[:3, 3] = [float(v) for v in pose["xyz"]]
    return matrix


def run(args):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from visualization_msgs.msg import MarkerArray

    chain = load_chain(find_urdf_root(args.urdf_root))
    record = load_records(args.input)[0]
    base = {name: float(record["joint_state"][name]) for name in JOINTS}
    summary(chain, record)

    rclpy.init()
    node = Node("q2_viz")
    joint_pub = node.create_publisher(JointState, "/joint_states", 10)
    marker_pub = node.create_publisher(MarkerArray, TOPIC, 10)

    start = time.monotonic()
    last_report = -1.0
    try:
        while rclpy.ok():
            elapsed = time.monotonic() - start
            if args.seconds > 0 and elapsed > args.seconds:
                break
            if args.animate:
                phase = 0.5 - 0.5 * math.cos(2.0 * math.pi * elapsed / args.period)
                alpha = smoothstep(phase)
            else:
                alpha = 0.0
            state = blend(base, DEMO_POSE, alpha)
            tip = objects_link_transform(chain, state)
            stamp = node.get_clock().now().to_msg()

            message = JointState()
            message.header.stamp = stamp
            message.name = list(JOINTS)
            message.position = [float(state[name]) for name in JOINTS]
            joint_pub.publish(message)

            markers, distance = build_markers(record, tip)
            stamp_markers(markers, stamp)
            marker_pub.publish(markers)

            if elapsed - last_report >= 2.0:
                last_report = elapsed
                print(f"t={elapsed:5.1f}s  alpha={alpha:.2f}  "
                      f"objects_link=({tip[0, 3]:.3f}, {tip[1, 3]:.3f}, {tip[2, 3]:.3f})  "
                      f"distance={distance:.3f} m", flush=True)
            time.sleep(1.0 / RATE_HZ)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description="题二 RViz 可视化（不产生答案）")
    parser.add_argument("--input", required=True, help="题二输入 jsonl")
    parser.add_argument("--urdf-root", default=None, help="官方 URDF 目录，缺省自动查找")
    parser.add_argument("--animate", action="store_true", help="在输入姿态与展示姿态之间往返")
    parser.add_argument("--period", type=float, default=8.0, help="往返周期（秒）")
    parser.add_argument("--seconds", type=float, default=0.0, help="运行时长，0 表示一直运行")
    run(parser.parse_args())


if __name__ == "__main__":
    main()
