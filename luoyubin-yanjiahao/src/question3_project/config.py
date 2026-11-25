#!/usr/bin/env python3
"""题三共享常量：场景 prim、抓取参数、运动帧数、视频与接触阈值。

各模块只从这里取参数，调参改一处即可；本文件不依赖 Isaac，导入安全。
"""
import sys
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parent
ENTRY_ROOT = PROJECT_DIR.parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

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
WIDE_GRASP_CLEARANCE = {"rubik_cube": 0.005, "soft_toy": 0.005}  # 掌部参考点停在中心上方的高度（米）
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
