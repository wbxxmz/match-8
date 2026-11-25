# 八界杯初赛作品（闫家豪、骆裕彬）

科沃斯「八界杯」开源机器人创新挑战赛 · 初赛作品。三道题各自独立可跑：题一在 RViz 里分步驱动关节，题二由 10 关节状态算夹爪与物体的位姿，题三在 Isaac Sim 里把夹爪送到三个玩具上并夹住。

| 项 | 内容 |
| --- | --- |
| 参赛者或团队 | 闫家豪、骆裕彬（团队） |
| 团队成员及分工 | 团队项目，共同完成三道题与复现材料 |
| 作品版本 | v1.0（2026-11-24） |
| 运行平台 | ROS 2 Humble（题一、题二）；Isaac Sim 4.5（题三） |

所有命令都在作品根目录执行：从比赛仓库根目录先 `cd yanjiahao-luoyubin`。题二不依赖 ROS 运行时，只需 Python 3.10、`numpy` 与 `xacro`；题三需要 Isaac Sim 4.5。

## 应用场景与目标

家庭服务机器人在桌面上取放小物件，前提是三件事：知道夹爪在哪、知道目标在哪、能把夹爪送到目标上。初赛三道题正好对应：题一是关节怎么动，题二是位姿怎么算，题三是算出来的位姿在物理仿真里能不能真的碰到东西。目标不是跑通一次，而是评委按本文复制命令能复现，数值能与官方金标对账。

## 主要功能与创新点

- **题一**：按 `seed` 采样展示姿态，50 Hz 插值分步驱动 10 个关节，RViz 同步显示；采样范围用离线 FK 校验过 5000 组，指尖不撞地、不进升降立柱。
- **题二**：不依赖第三方 FK 库，自己展开官方 xacro 建链、读每个关节的 origin/axis、做链式正解；与官方 `demo_zero` 金标 6 位小数一致，并做正逆往返与 rpy/quat 一致性校验。
- **题三**：复用题二的关节链与 FK，补阻尼最小二乘逆运动学、底盘锁定、分段缓动与接触判据（PhysX 接触报告、触底回升、指关节力矩、玩具位移）；按玩具形状选抓取姿态——魔方与软玩具竖直宽抓、恐龙竖直侧抱。

## 技术方案

三题共用一条“关节状态 → 正运动学 → 位姿”的主线，题三在其上加逆解与控制。系统结构、公式推导、控制流程、判据阈值与完整实测数据见 [PROJECT.md](PROJECT.md)；本文只给运行方式与验证结果。

## 运行环境与依赖

| 组件 | 版本 | 说明 |
| --- | --- | --- |
| Ubuntu | 22.04 | |
| ROS 2 | Humble | 题一需要 `robot_state_publisher`、`rviz2`；题二只用 `xacro` 展开模型 |
| Python | 3.10 | 题一仅用标准库，题二用 `numpy` |
| numpy | 2.2.6 | 题二矩阵运算；`pip install numpy` 或系统包均可 |
| Isaac Sim | 4.5.0-rc.36 | 题三仿真环境，安装路径自定；本机在 `/mnt/hbotdata/tmp/isaac-sim` |
| opencv-python | 4.12 | 仅题三 `--video` 录制用；不录制视频可不装 |
| 官方模型 | `urdf/bajie_robot.urdf.xacro` | 题一、题二都按 `BAJIE_URDF_ROOT` > 从作品目录逐级向上找 `urdf/` 的顺序定位（题二另有 `--urdf-root` 与作品内置副本 `src/question2_project/model/` 兜底）；找不到时题一会直接提示设置 `BAJIE_URDF_ROOT`，不会甩一个临时路径报错 |

```bash
sudo apt-get install -y ros-humble-robot-state-publisher ros-humble-rviz2 ros-humble-xacro
source /opt/ros/humble/setup.bash
```

以下所有命令默认当前目录为作品根目录 `yanjiahao-luoyubin/`（克隆比赛仓库后先执行 `cd yanjiahao-luoyubin`）。

## 目录

```
yanjiahao-luoyubin/
├── README.md
├── PROJECT.md                # 题二设计说明（FK 推导与验证）
├── src/question1_project/
│   ├── q1_driver.py          # 关节分步驱动与关键帧输出
│   ├── robot_urdf.py         # 展开官方 xacro，替换 package:// 路径
│   ├── config/joint_limits.json
│   ├── config/q1.rviz
│   ├── launch/q1.launch.py
│   └── tests/                # 驱动单测 + 0~127 全种子校验回归
├── src/question2_project/
│   ├── urdf_chain.py         # 展开官方 URDF，抽取 base_footprint → objects_link 链
│   ├── transforms.py         # rpy ↔ 矩阵 ↔ 四元数、逆变换
│   ├── fk.py                 # 链式正向运动学
│   ├── run_q2.py             # 读输入、算结果、写输出
│   ├── viz_q2.py             # 可选 RViz 可视化（不产生答案）
│   ├── requirements.txt      # 运行依赖（numpy）
│   ├── model/                # 官方 xacro 源码副本（兜底用，不含 mesh）
│   ├── config/q2.rviz
│   ├── launch/q2.launch.py
│   ├── tests/test_gold.py    # 金标、语义不变量与自检用例
│   └── tests/test_cross_validation.py # PyKDL 第二套实现交叉校验（含随机姿态）
├── src/question3_project/
│   ├── touch_toy.py          # 入口：解析参数、串起各模块、写答案
│   ├── config.py             # 常量：prim、抓取参数、运动帧数、视频与接触阈值
│   ├── input_io.py           # 场景定位、输入校验、环境证据、测例过滤
│   ├── contact.py            # 触碰判定口径与 PhysX 接触报告订阅
│   ├── sim_runtime.py        # Isaac Sim 启动、物理推进、底盘锁定、录像
│   ├── arm.py                # 关节动作原语：插值、过渡、下降、合拢
│   ├── planning.py           # 目标点选择、多初值 IK、干跑预检
│   ├── case_runner.py        # 单个测例的执行与答案组装
│   ├── planner.py            # 底盘坐标变换、逆运动学、位姿换算（不依赖 Isaac）
│   ├── verify_q3.py          # 离线自检：答案/FK/闭环/日志/视频，不依赖 Isaac
│   ├── tests/test_planner.py # 规划回归测试：变换闭环、FK、IK 残差、限位与另一组初值
│   └── tests/test_touch_logic.py # 触碰判定口径与输入校验（未知玩具、缺字段、非有限位姿）
├── inputs/q1_demo_input.jsonl
├── inputs/q2_demo_input.jsonl
├── inputs/q3_demo_input.jsonl
├── answers/q1_demo_output.jsonl
├── answers/q2_demo_output.jsonl
├── answers/q3_demo_output.jsonl
├── docs/                     # 截图等补充材料
├── assets/                   # 终端日志
└── video/                    # 演示视频（单个文件不超过 30 MB）
```

## 题目一：关节分步驱动与 RViz 展示

### 复现步骤

一条命令启动 RViz 并自动跑完整流程（预热 8 秒后开始动作）：

```bash
source /opt/ros/humble/setup.bash
ros2 launch src/question1_project/launch/q1.launch.py
```

启动后会依次看到：零位 → 升降 → 头部俯仰 → 机械臂摆到展示姿态 → 夹爪张开 → 合拢（停在展示姿态）。
驱动同时在终端打印关键帧，并把答案写回 `answers/q1_demo_output.jsonl`。

想手动控制驱动（先在终端 A 启动 RViz，再在终端 B 发指令）：

```bash
# 终端 A（作品根目录 yanjiahao-luoyubin/）
source /opt/ros/humble/setup.bash
ros2 launch src/question1_project/launch/q1.launch.py driver:=false

# 终端 B（作品根目录 yanjiahao-luoyubin/）
source /opt/ros/humble/setup.bash
python3 src/question1_project/q1_driver.py \
  --input inputs/q1_demo_input.jsonl \
  --output answers/q1_demo_output.jsonl \
  --ros
```

只想生成答案、不开 RViz 时去掉 `--ros`，几秒即可跑完。

### 预期输出

`answers/q1_demo_output.jsonl` 应只有 1 行。`seed=7` 时各关键帧的主要数值如下（其余关节按序保持上一帧的值）：

| 关键帧 | lift_joint | pitch_joint | joint1 | joint2 | joint5 | 手指 |
| --- | --- | --- | --- | --- | --- | --- |
| lift | 0.17829 | 0 | 0 | 0 | 0 | 0 |
| head | 0.17829 | -0.28966 | 0 | 0 | 0 | 0 |
| arm | 0.17829 | -0.28966 | 0.301869 | -0.456538 | -0.553601 | 0 |
| gripper_open | 同上 | 同上 | 同上 | 同上 | 同上 | 0.506 |
| gripper_close | 同上 | 同上 | 同上 | 同上 | 同上 | 0 |

`final_joint_state` 与 `gripper_close` 帧完全相同。换输入里的 `seed` 后数值会变，但结构与上表一致。

### 自检

```bash
python3 src/question1_project/check_output.py
```

预期输出：

```
[PASS] 第 1 行 'q1_all_seed7'：5 帧顺序、10 关节、限位、阶段隔离、mimic 与最终态均一致
自检通过：1 行答案全部符合题面要求
```

脚本会逐项检查：答案行数与 `id` 是否对应输入、五帧顺序、每帧 10 个关节、数值是否有限且在限位内、夹爪开合值、`final_joint_state` 是否等于末帧，以及三条容易漏的规则：阶段隔离（非本阶段关节必须与上一帧逐位相同）、本阶段至少一个关节真的动了、手指 mimic 关系（系数与偏置直接从官方 URDF 读取，不用手抄）。任何一项不符会打印 `[FAIL]` 并以退出码 1 结束。

单元测试：

```bash
python3 -m pytest -q src/question1_project/tests
```

预期输出 `13 passed`。覆盖：同 seed 确定性、换 seed 变化、采样值在限位内、限位校验拒绝越界/未知/缺失关节、五段结构与末帧一致、输入错误报行号、非有限时长/小数 seed/非法 step 被拒，以及 0~127 每颗种子的答案逐个过一遍正式校验器（128 颗种子至少 120 种不同目标，防止 seed 分支退化）。

### 行为说明

- `step` 固定为 `all`，流程顺序 `lift → head → arm → gripper_open → gripper_close`。
- 动作从官方零位（全部关节为 0）开始，五段依次执行，`seed` 决定展示姿态，10 个关节全部参与：
  1. **lift**：升降关节抬到 seed 采样的高度（0.12~0.30 米）；
  2. **head**：头部俯仰到 seed 采样的角度（-0.35~0.05 弧度）；
  3. **arm**：`joint1`~`joint6` 全部按 seed 采样，一次摆到展示姿态；
  4. **gripper_open**：两指张开到限位 0.506 弧度；
  5. **gripper_close**：两指合拢，停在展示姿态（不回零）。
- 采样范围经过离线 FK 校验（5000 个 seed）：指尖始终在身前 0.87 米外、高度 0.29 米以上，各连杆不进入升降立柱，不撞地。
- 同一个 `seed` 输出完全一致；改输入里的 `seed` 会得到不同姿态，可直接对比。
- `move_sec` 内 50 Hz 平滑插值（smoothstep 缓动，起停速度为 0），到位后保持 `pause_sec` 再采样。
- 每帧发布完整 10 个关节；两个手指关节成对发布。
- 所有目标值发布前用 `config/joint_limits.json` 校验（该文件由官方 URDF 的 `limit` 字段生成），越界直接报错。
- ROS 模式先保持官方零位预热（`--warmup`，launch 默认 8 秒）再开始动作，消除 RViz 收到首帧前的一次性跳变。
- 输出沿用输入 `id`；`keyframes` 每项含 `name` 与 10 关节 `joint_state`；`final_joint_state` 与最后一个关键帧一致。

### 与官方 launch 的差异（两个实测易错点）

本作品自带 `launch/q1.launch.py`，不直接使用官方 `display_bajie_robot_urdf.launch.py`。按官方 launch 自行组装环境时，实测会遇到：

1. **`use_sim_time` 默认是 `true`**：官方 launch 第 15 行把该参数默认设为 `true`，但没有仿真时钟 `/clock` 时，`rviz2` 按仿真时间等待，时间轴不走、画面像卡住，容易误判成“驱动没生效”。用真实时间必须显式传 `use_sim_time:=false`；本作品 launch 不启用仿真时间。
2. **`joint_state_publisher_gui` 会和驱动抢 `/joint_states`**：官方 launch 第 47-49 行启动的滑块窗口是独立节点，不是 RViz 内置面板；它与本作品的驱动脚本同时向 `/joint_states` 发布，两个发布者互相覆盖，表现为画面抖动或姿态被拉回零位。本作品 launch 不启动该节点；如需手动拖滑块调试，先停掉驱动脚本。

### 验证

```bash
python3 -c "import json; [json.loads(l) for l in open('answers/q1_demo_output.jsonl')]"
bash ../validate-entry-submission.sh yanjiahao-luoyubin
```

### 素材

| 文件 | 内容 |
| --- | --- |
| `assets/q1.log` | 完整终端日志：launch 与 RViz 启动、环境证据（OS / Python / ROS_DISTRO / git commit）、驱动五段关键帧输出（53 行，0 条 error） |
| `video/q1_rviz.mp4` | RViz 演示视频：48 秒、1920×1200、15 fps，mp4（MPEG-4），全屏录制，覆盖零位到夹爪闭合的完整流程，左侧 Displays 面板文字可读 |

## 题目二：正向运动学与坐标变换

### 复现步骤

```bash
pip install -r src/question2_project/requirements.txt   # 仅 numpy
python3 src/question2_project/run_q2.py \
  --input inputs/q2_demo_input.jsonl \
  --output answers/q2_demo_output.jsonl
```

不需要 ROS 运行时。脚本自动调用 `xacro` 展开官方模型：先找 PATH 里的 `xacro`，找不到时用 `/opt/ros/humble/bin/xacro`，未 `source` ROS 环境也能跑。模型目录按 `--urdf-root` > `BAJIE_URDF_ROOT` > 从作品目录逐级向上找 `urdf/` > 作品内置 `src/question2_project/model/` 的顺序定位；评审环境即使没有克隆官方 URDF 仓库也能直接跑。

### 预期输出

终端先打印关节链，再写 1 行答案：

```
关节链：base_joint → lift_base_joint → lift_joint → arm_base_joint → joint1 → joint2 → joint3 → joint4 → joint5 → joint6 → gripper_joint → object_joint
已写入 1 行 -> answers/q2_demo_output.jsonl
```

`answers/q2_demo_output.jsonl` 与官方金标一致：

| 字段 | xyz | rpy | quat_xyzw |
| --- | --- | --- | --- |
| `objects_link_in_base_footprint` | `[1.103715, -0.062685, 0.629136]` | `[0.0, 0.000832, 0.0]` | `[0.0, 0.000416, 0.0, 1.0]` |
| `scene_object_in_objects_link` | `[-0.603441, -0.437315, -0.329638]` | `[0.0, -0.000832, 0.0]` | `[0.0, -0.000416, 0.0, 1.0]` |

### 自检

```bash
python3 -m pytest -q src/question2_project/tests
```

预期输出 `18 passed`。覆盖：关节链顺序（含头部/手指不在链上）、两个位姿与金标比对（`xyz` 差 `< 1e-4`）、正逆往返误差 `< 1e-6`、`rpy`/`quat_xyzw` 互转一致 `< 1e-6`、内置模型副本独立复现金标、缺关节或未知关节报错、位姿含 `nan`/`inf` 报错、答案逐行可解析且与重算一致；另加三条语义不变量（头部/手指等无关关节不影响输出、`lift_joint` 只沿 z 平移且姿态不变、`joint6` 只改姿态不改位置）。

官方只提供 `demo_zero` 一组金标，零位对得上不代表整条链对。为此另有一套 PyKDL 独立交叉校验：自己从 URDF 回溯关节链、自己构造 KDL 链（不复用产品代码的建模逻辑），覆盖零位、逐个关节、每个关节的限位边界、100 组随机姿态、20 次物体坐标变换，位置与姿态矩阵误差阈值 `1e-9`（实测 `1e-16` 量级）。缺 PyKDL 时该组测试自动跳过并在输出里说明。

### 可选可视化（不影响答案）

```bash
source /opt/ros/humble/setup.bash
ros2 launch src/question2_project/launch/q2.launch.py
```

RViz 中橙色方块为场景物体，红色圆点为 `objects_link`，三色箭头为 `objects_link` 的三轴，黄色连线长度为两者距离。节点在输入姿态与一个演示姿态之间往返（周期 `period` 默认 8 秒），可观察关节角变化对正运动学的影响。该演示只用于观察，答案仍由 `run_q2.py` 生成；`answers/` 不会因此改变。可视化需要外部官方 `urdf/` 目录（内置副本不含 mesh）。

### 实现说明

- 关节原点与轴向全部从 xacro 展开后的 URDF 读取，不硬编码；固定关节也保留在链上。
- 转动关节绕 URDF 给定的 `<axis>` 旋转，`lift_joint` 是移动关节（沿 z 平移，单位米）。
- 输入必须是完整 10 关节；缺关节、未知关节、非数值直接报错，不静默补零。
- 场景物体用逆变换 `Rᵀ`、`-Rᵀt` 从底盘系变换到夹爪系，不做数值求逆。
- `rpy` 从旋转矩阵按 Z-Y-X 反解；`quat_xyzw` 顺序 `[x, y, z, w]`，统一取正半球。
- 输出保留 6 位小数，`-0.0` 归一为 `0.0`。

### 素材

| 文件 | 内容 |
| --- | --- |
| `assets/q2.log` | 完整终端日志：运行命令、环境证据（OS / Python / numpy / ROS / git commit）、模型目录、关节链、写入结果与退出码 0 |
| `assets/q2_viz.log` | 可视化终端日志：launch、RViz 与演示节点输出（63 行，0 条 error） |
| `video/q2_rviz.mp4` | 22 秒、1920×1200、15 fps，mp4（MPEG-4），全屏录制；机械臂在零位与演示姿态间往返，距离由 0.753 米变到 0.805 米 |

### 内置模型副本来源

`src/question2_project/model/` 是官方 URDF 资源仓库 `https://atomgit.com/ecovacs-bajie/ecovacs-bajie-urdf`（本地克隆 commit `907b0611eb3cf7dd32b410b8ae32d3ac7ffca775`）的逐字节副本，只保留 FK 必需的 xacro 与标定文件，不含 mesh，未修改任何 origin 或轴向。

| 文件 | SHA256 |
| --- | --- |
| `urdf/bajie_robot.urdf.xacro` | `6dc26b06aafb82aa7030eabf54e5c181b17b4173eb8fc5e64103fbcbdb0d8f49` |
| `urdf/xacro_property.urdf.xacro` | `589799ae15573bff2ba9dfdd5f115d6b0cde879f957dbf2775d2c511816c8852` |
| `urdf/gripper.urdf.xacro` | `802e7ef633d4cedbbb8f4e9f3d48b36a3b961bf3b27229347a40507d4b8744d3` |
| `urdf/berxel_arm_base_link.urdf.xacro` | `8f3deca4c91b4ed881ef9b0c2745600355fcb01264a11cc4efbc2cb520a71cd7` |
| `urdf/berxel_upper_base_link.urdf.xacro` | `c8879a90a53ae03fbacefcd35a35de840a51d290ea2c3072af8d5d37eb5811dc` |
| `calib/calibration.yaml` | `34eb12810602dfd79b50b6fec4d813817bfb8e69e375207d954e43eb71bebcf3` |

## 题目三：Isaac Sim 中触碰三个玩具

### 复现步骤

需要 Isaac Sim 4.5（官方场景 `sim/scene/abr_scene.usd`）与 ROS 2 Humble。所有命令在作品根目录 `yanjiahao-luoyubin/` 执行：

```bash
source /opt/ros/humble/setup.bash
export OMNI_KIT_ACCEPT_EULA=YES ROS_DISTRO=humble
/mnt/hbotdata/tmp/isaac-sim/python.sh src/question3_project/touch_toy.py \
  --scene /home/yjh/bajie-competition/sim/scene/abr_scene.usd \
  --input inputs/q3_demo_input.jsonl \
  --output answers/q3_demo_output.jsonl \
  --headless
```

`python.sh` 换成自己的 Isaac Sim 4.5 安装路径即可。加 `--video video/q3_touch.mp4` 会在无窗口模式下用独立相机同时录制触碰视频（需要 `opencv-python`）；去掉 `--headless` 则打开 Isaac Sim 图形窗口，视口自动对准三个玩具，可实时观看；`--scene` 省略时按 `BAJIE_SIM_ROOT`（指向 `sim/` 仓库根目录）或从作品目录逐级向上查找 `sim/scene/abr_scene.usd`。

### 离线预检（不需要 Isaac）

只用输入里的底盘与玩具位姿验证坐标换算与 IK 可达性，不启动仿真、不写答案，普通 `python3` 即可：

```bash
python3 src/question3_project/touch_toy.py --dry-run
```

输出每个玩具的预抓取点与抓取点的位置/姿态残差、是否穿地，3 个测例 × 2 个点全部标 `可达` 时退出码为 0。只想看一个玩具加 `--only rubik_cube`（id 或玩具名都行）；`--only` 默认不写答案，避免半份答案覆盖正式文件，需要写时显式加 `--output`。

`--finger-gap <弧度>` 控制闭合终点保留的指关节角，默认 0；留缝会削弱夹持力矩，正式答案保持 0。

### 实现路径

不走 ROS 话题收发，直接用 Isaac 的 `Articulation` 接口下发关节目标（Bridge 仍按官方场景要求启用，否则场景里的 ROS 2 OmniGraph 节点会报错）。这样控制回路不依赖 DDS 时序，答案可复现，也不需要额外的控制器节点。

这台机器人是轮式自由 articulation，机械臂伸展时的反作用会把整台车推走（实测可达 33 厘米），答案位姿就和真实位姿错开。该 Isaac 版本没有 `fixRootJoint`，所以程序在初始化后记下底盘世界位姿，之后每个物理步都把它写回，等价于把底盘钉在原地（日志里的 `BASE_LOCK_ON` 与每例末尾的“底盘位移”可核对）。

每个测例的流程：

1. 读入 10 关节初值，用 `set_joint_positions` 置入，等待 40 个物理步后读回并打印误差；
2. 由 `base_footprint_in_world` 与玩具世界位姿求玩具在底盘系的坐标，与输入给的 `scene_object_in_world` 互相核对；
3. 阻尼最小二乘逆运动学（复用题二的 URDF 链与 FK）求预抓取点与抓取点。`objects_link` 的 **+X 是接近方向**（从手腕指向指尖）、+Y 是两指张开方向、+Z 是手指转轴；三个姿态都按这个口径构造：魔方与软玩具竖直朝下、两指张到 0.5 弧度，掌部参考点分别停在中心上方 5 毫米与 20 毫米；恐龙竖直朝下、两指沿短轴（6.6 厘米）张开，停在中心上方 2 厘米；
4. 分段插值下发目标：抬升与转向合成一段——IK 求玩具方位、半径 0.40 米、高 0.30 米的转运姿态（高于所有玩具；转运时二指保持合拢），到位后再下降到预抓取。折叠状态下直接转底座会让前臂从机身里扫过去（官方场景关闭了自碰撞，画面上就是穿模）；在玩具高度上转又会把三个玩具扫飞，所以转到 0.30 米高度完成；转运后等夹爪真的到位（误差 < 2 厘米）再下降；
5. 下降到玩具后合拢两指夹住：三个玩具都是从两侧包住再合拢；下降途中仍监测触底回升与指力矩，任一达标即停止，避免把玩具推走；
6. 读回最终关节，写一行 JSONL。

测例之间不做瞬移。上一例接触结束后，程序先带着玩具微抬 2 厘米再松指（原地张开会把玩具横向推走），然后只升降抬到 0.30 米转运高度，在这个高度上把姿态平滑插值到下一例初值，最后单独把升降降到位并等物理收敛；剩下的跟随静差再用 30 帧分帧补齐，使下一例的“置初值”不再产生可见跳变。过渡动作不属于任何测例，期间的接触记录暂停；每段过渡后打印三个玩具的位移，便于核对有没有被碰动。

### 坐标与场景映射

| 项 | 值 |
| --- | --- |
| 机器人 articulation root | `/World/abr_machine/robot_root/base/base_footprint` |
| 夹爪参考系 | `/World/abr_machine/robot_root/gripper/objects_link` |
| `rubik_cube` 魔方 | `/World/company/toys/Rubik` |
| `dinosaur` 恐龙 | `/World/company/toys/toy_yellow_dragon` |
| `soft_toy` 软玩具 | `/World/company/toys/spiderman_grasp` |

映射是按输入给的玩具世界坐标与场景实际坐标逐一对出来的，三个测例差值分别为 0.0001、0.0019、0.0000 米。

**恐龙资产自带 -90° 旋转，抓取前先摆正到题面位姿**：`toy_yellow_dragon` 在场景里带 `xformOp:rotateXYZ=(0, 0, -90)`（缩放 0.2），题面输入记的却是 `rpy=[0,0,0]`。题目要求按输入位姿抓取，所以每个测例开始时先把目标玩具刚体设成输入的世界位姿（位置 + 朝向，读回误差 ≤2 毫米 / 0.0°），等物理收敛 0.5 秒再规划；恐龙因此转正，长轴顺着三个玩具的排布方向。摆正后它是 23 厘米的长条，因此恐龙用**竖直侧抱**：夹爪从正上方下，两指沿短轴（6.6 厘米）张开到 0.5 弧度，停在身体中心上方 2 厘米再合拢，两指从身体两侧包住躯干，不往下压。魔方的两指沿底盘 x 轴张开、贴合方块侧面；软玩具的脸朝侧面，两指改沿 y 轴张开，两片指面正好贴在脸的两颊上、从左右夹住正脸再合拢。答案主字段按输入位姿计算；仿真实际位姿另存 `scene_object_in_objects_link_sim`，两者差异只来自夹取时的推动：魔方 1.6 毫米 / -0.1°，恐龙 1.5 毫米 / -0.2°，软玩具 4.3 毫米 / +0.5°。

**夹爪坐标口径修正**：`objects_link` 的 +X 才是接近方向，早期版本把它当成 +Z，所谓“竖直朝下”实际是横着伸过去；同时 IK 的旋转误差用反对称部分，在 180° 处恰好归零，把手腕反了的解当成已对准。两处叠加就是“关节扭反、斜着抓”。现在旋转误差改成轴角公式（180° 也能分辨）、雅可比按误差向量差分，三个姿态统一按 +X 构造；用最终关节反算的接近方向是 `[-0.001, 0.001, -1.000]`，三例都真正竖直向下。

### 触碰判据

`objects_link` 距法兰盘 0.12236 米，位于两指末端、靠近指尖的抓取参考点，所以“到玩具中心的距离”要结合玩具尺寸看，不能单独证明碰到。程序同时记录三个物理量，任一满足即判定触碰：

| 判据 | 阈值 | 说明 |
| --- | --- | --- |
| 下降触底回升 | 距离进入 6 厘米后，比最优值回升 3 毫米 | 宽抓/侧抱下降时若指力矩还没到阈值，也用回升兜底；远处手腕摆动造成的回升不算 |
| 指关节力矩 | > 0.15 N·m | 手指被玩具挡住；下降途中达到该值也立即停止，避免继续下压把玩具推走 |
| 玩具位移 | > 5 毫米 | 玩具被推离原位，只有机器人能造成 |
| PhysX 接触报告 | 分离 ≤ 1 毫米、冲量 > 1e-6 N·s | 仿真器直接上报“哪个连杆碰到了哪个玩具”，是触碰的客观证据 |

斜抓（旧方案）时自上而下压住玩具，力沿指连杆方向、不绕指关节轴，指力矩测不到，所以要配合“触底回升”和“玩具位移”；现在三个玩具都是两指从左右两侧包住再合拢，指力矩是主要证据（实测 0.276 / 0.676 / 0.651 N·m）。前三项是间接推断，第四项是硬证据：程序给机器人连杆与三个玩具挂 `PhysxContactReportAPI`（阈值 0），订阅接触事件，打印碰撞对、分离距离与冲量，并检查是否碰到目标以外的玩具。距离不再单独判 PASS，只作为几何参考打印出来——距离近不等于碰到。

### 预期输出

终端打印每个测例的初值读回误差、玩具坐标核对、IK 残差、每段距离、PhysX 接触对与判据，最后写 3 行答案。用本仓库录制视频的那次运行（`assets/q3.log`）得到的数值：

| id | 玩具 | 最终距离 | 峰值指力矩 | 玩具位移 | PhysX 接触对（分离 / 冲量） | 结果 |
| --- | --- | --- | --- | --- | --- | --- |
| `touch_toy_1` | 魔方 | 0.0046 m | 0.276 N·m | 0.7 mm | `gripper_left_link` ↔ `Rubik`（0.030 mm / 0.000279 N·s） | PASS |
| `touch_toy_2` | 恐龙 | 0.0194 m | 0.676 N·m | 0.3 mm | `gripper_right_link` ↔ `toy_yellow_dragon`（0.064 mm / 0.000377 N·s） | PASS |
| `touch_toy_3` | 软玩具 | 0.0196 m | 0.651 N·m | 4.2 mm | `gripper_right_link` ↔ `spiderman_grasp`（0.223 mm / 0.000142 N·s） | PASS |

三个测例的初值读回最大误差都不超过 0.000055 rad/m，USD 位姿与 FK 重算偏差最大 0.12 毫米，底盘位移 0.0000 米（已锁定）。物理仿真对接触细节敏感，重跑时数值会小幅变化，但三个玩具都会被执行到。

`answers/q3_demo_output.jsonl` 每行字段：

| 字段 | 含义 |
| --- | --- |
| `id` | 沿用输入 |
| `final_joint_state` | 最终 10 关节状态 |
| `objects_link_in_base_footprint` | 夹爪参考系在底盘系的位姿 |
| `scene_object_in_objects_link` | 玩具在夹爪参考系的位姿（题面要求的“玩具相对于夹爪”报告），按输入 `scene_object_in_world` 计算，与题面 2.4 节口径一致 |
| `scene_object_in_objects_link_sim` | 同一报告，但按仿真里的实际位姿计算（摆正后的初始位姿与触碰推动都包含在内），供按仿真复算时对账 |
| `scene_object_input_gap_m` / `scene_object_input_gap_yaw_deg` | 上面两种口径的差异：位置（米）与偏航（度）；本机为魔方 1.6 毫米 / -0.1°、恐龙 1.5 毫米 / -0.2°、软玩具 4.3 毫米 / +0.5° |
| `touch_distance_m` / `min_distance_m` | 最终/过程中夹爪参考点到玩具中心的最小距离 |
| `contact_detected` | 触碰判据是否满足 |
| `finger_effort_nm` | 峰值指关节力矩 |
| `toy_displacement_m` | 玩具被推离原位的距离 |
| `physx_contact` | PhysX 上报的目标碰撞对：碰撞的两个 prim、接触点世界坐标、冲量、分离距离；无上报时为 `null` |
| `unexpected_contact` | 触碰目标以外玩具的事件（正常为 `null`） |
| `initial_state_error_max` | 初值写回最大误差 |

### 自检

```bash
python3 src/question3_project/verify_q3.py
python3 -m pytest src/question3_project/tests -q
```

第一条检查产物：答案行数与 id、10 关节与限位、FK 重算、玩具位姿闭环（主字段必须回到输入位姿，容差 2 毫米；仿真实测字段允许“玩具位移 + 2 毫米”）、接触证据、日志与视频；同时做环境预检（Python/numpy 版本、URDF 关节链、场景文件、Isaac python.sh），缺场景或 Isaac 只给 WARN，不影响离线校验。第二条是规划回归测试：位姿变换闭环、答案与 FK 一致、三个玩具中心的 IK 残差、以及一组不同初值的 FK 与限位检查。

预期输出：

```
ENV   Python 3.10.12，numpy 2.2.6
ENV   关节链 12 个关节加载成功
ENV   场景 /home/yjh/bajie-competition/sim/scene/abr_scene.usd
ENV   Isaac python /mnt/hbotdata/tmp/isaac-sim/python.sh
OK    touch_toy_1: 距离 0.0046 m（上限 0.0502），力矩 0.277 N·m，位移 0.7 mm，FK 差 0.000000 m，主字段闭环差 0.000001 m，仿真实测闭环差 0.001586 m
OK    touch_toy_2: 距离 0.0194 m（上限 0.1290），力矩 0.676 N·m，位移 0.3 mm，FK 差 0.000000 m，主字段闭环差 0.000001 m，仿真实测闭环差 0.001508 m
OK    touch_toy_3: 距离 0.0196 m（上限 0.0690），力矩 0.651 N·m，位移 4.2 mm，FK 差 0.000000 m，主字段闭环差 0.000001 m，仿真实测闭环差 0.004336 m
PASS  题三答案、日志与视频自检通过
```

回归测试应输出 `22 passed`。Python 小版本、numpy 版本与场景/Isaac 路径随机器不同，属正常差异。

距离随物理细节小幅浮动，`contact_detected` 恒为 `True`。跑完会打印三行结果汇总；图形模式默认再保持窗口 90 秒，方便看清最后一个玩具的最终姿态，可用 `--hold-sec 0` 关闭或改时长。

**重跑差异**：同一套方案的不同次 headless 运行里，魔方约 0.0047~0.0055 米、恐龙约 0.019~0.026 米、软玩具约 0.004~0.020 米，差异主要来自“首次达到力矩阈值就停”的位置不同；三例都满足判据，且每次都有 PhysX 接触对上报。夹爪坐标口径修正后未再做 GUI 对照，正式数值以 headless（录视频）为准；此前旧姿态方案的 GUI/headless 对照只用于确认判定口径一致（距离最大差 0.3~0.4 毫米、峰值力矩最大差 0.198 N·m、玩具位移最大差 1.4 毫米）。判定现在以 PhysX 接触报告为硬证据，渲染节拍不参与判定。

### 素材

| 文件 | 内容 |
| --- | --- |
| `assets/q3_env.log` | 环境证据（`--env-only` 只打印环境、不启动仿真，3 秒内退出）：Linux 6.8 / Python 3.10.15 / numpy 1.26.0 / git commit / RTX 4060 Laptop + 驱动 580.178.04 / Isaac Sim 4.5.0-rc.36+release.19112；该模式不加载 ROS，故 `ROS_DISTRO` 显示 unset |
| `assets/q3.log` | 完整终端日志：场景加载、底盘锁定、三个测例的初值误差、坐标核对、玩具包围盒、IK 残差、转运姿态、到位误差、距离曲线、判据、两种口径的玩具位姿差、PhysX 接触对（分离距离与冲量）、测例间过渡的收敛残差与玩具位移、底盘位移、结果汇总与写入（109 行，0 条 error/warning；Isaac 引擎自身的启动信息与告警已省略，程序输出全保留） |
| `video/q3_touch.mp4` | Isaac Sim 触碰视频：1280×720、15 fps、60.0 秒（900 帧），mp4（MPEG-4），整条机械臂与三个玩具均入镜，可见竖直下降、两指夹住玩具，以及测例之间“松指—抬升—高位转向—下降”的连续过渡 |
| `video/q3_touch_h264.mp4` | 同一段录制的 H.264 版本（900 帧、60.0 秒、6.2 MB）：原始文件为 `mp4v` 编码，部分编辑器/浏览器播放器只认 H.264，遂无损时间轴转码一份便于直接播放；画面经过有损压缩，接触帧号与上表一致 |
| `docs/q3_contact_sheet.png` | 三行接触截图（第 202 / 550 / 882 帧）：每行左侧一张 800×450 整帧画面，右上角红框内为同帧放大特写（对准夹持点），标注帧号、时间、玩具名、距离与指力矩，可直接看到两指夹住魔方 / 恐龙 / 软玩具 |

视频关键时间点（`mp4` 可直接跳转）：

| 时间 | 画面 | 本刻证据 |
| --- | --- | --- |
| 00:03 | 抬升并转向，手臂斜向上到 0.30 米 | 不折回机身 |
| 00:13 | 两指夹住魔方（第 202 帧） | 距离 0.0046 米，峰值力矩 0.276 N·m，`gripper_left_link` ↔ `Rubik` |
| 00:37 | 竖直侧抱夹住恐龙（第 550 帧） | 距离 0.0194 米，峰值力矩 0.676 N·m，`gripper_right_link` ↔ `toy_yellow_dragon` |
| 00:59 | 夹住软玩具（第 882 帧） | 距离 0.0196 米，峰值力矩 0.651 N·m，`gripper_right_link` ↔ `spiderman_grasp` |

录制视频的命令在上面基础上加 `--video video/q3_touch.mp4`；相机位置与注视点固定在 `touch_toy.py` 顶部的 `VIDEO_EYE` / `VIDEO_TARGET`。视频**未加速、未抽时**：物理步长为 Isaac 默认 1/60 秒，每 4 个物理步渲染一帧、按 15 fps 写出，4 × 1/60 = 1/15 秒，与仿真时间 1:1（900 帧 = 3602 步 = 60.0 秒）。接触截图直接由这份视频抽帧生成（视频帧号 = 物理步数 ÷ 4），抽帧脚本不随作品提交。三个接触帧号是逐帧核验过的：先按运行时打印的判定接触物理步数（806 / 2200 / 3602 步）定位区间，再逐帧放大核对指尖与玩具的间隙，取夹持稳定且玩具未被手指完全挡住的一帧，最终取 202 / 550 / 882。软玩具一格使用的是两指从两颊包住正脸、还没抬起的一帧；日志里同一时刻的距离是 0.0196 米、合拢力矩 0.651 N·m。注意帧号只对 `video/q3_touch.mp4` 这份视频有效，换一次重跑时序会变。

### 资源与风险

| 项 | 说明 |
| --- | --- |
| 显存 | 官方最低 8 GB，实测 8 GB 可跑 headless；开 GUI 前建议关掉占显存的程序 |
| 内存 | Isaac 常驻 8~12 GB，16 GB 机器上不要同时开浏览器与其它大程序 |
| 磁盘 | Isaac Sim 本体不要装在根分区，答案与日志很小可直接进仓库 |
| 运行耗时 | headless + 录视频一次约 235 秒（8 GB 显存，三个测例含物理稳定、测例间过渡与保持等待）；图形模式另加 `--hold-sec` 的 90 秒 |
| 不确定性 | 接触瞬间的摩擦与穿透由 PhysX 决定，重跑数值会小幅变化；判定用阈值而非固定值。同代码连续两次重跑距离差 ≤ 0.3 毫米，不同转运方案间最大差 1.3 cm |

## 演示与验证结果

| 题 | 复现命令（作品根目录） | 预期结果 | 材料 |
| --- | --- | --- | --- |
| 一 | `python3 src/question1_project/check_output.py`、`python3 -m pytest -q src/question1_project/tests` | 打印 `[PASS]`；用例全部通过 | `assets/q1.log`、`video/q1_rviz.mp4` |
| 二 | `python3 -m pytest -q src/question2_project/tests` | 用例全部通过（与官方 `demo_zero` 差 < 1e-6） | `assets/q2.log`、`assets/q2_viz.log`、`video/q2_rviz.mp4` |
| 三 | `python3 src/question3_project/verify_q3.py`、`python3 -m pytest -q src/question3_project/tests` | 三个测例 PASS；用例全部通过 | `assets/q3.log`、`assets/q3_env.log`、`video/q3_touch.mp4`、`docs/q3_contact_sheet.png` |

题三实测（headless，每次重跑数值有小幅浮动）：魔方最终距离 0.0046 米、指力矩 0.276 N·m；恐龙（竖直侧抱）0.0194 米、0.676 N·m；软玩具（两指沿 y 夹住两颊）0.0196 米、0.651 N·m。三例均满足触碰判据，且都有 PhysX 接触对上报。视频与截图都是真实运行录制，未做任何合成。

## 文件说明

| 路径 | 作用 |
| --- | --- |
| `src/question1_project/` | 题一驱动、限位配置、RViz 配置与启动文件、自检与用例 |
| `src/question2_project/` | 题二 FK 实现（`urdf_chain.py`、`transforms.py`、`fk.py`）、读写入口、可选可视化、内置模型副本与用例 |
| `src/question3_project/` | 题三规划（`planner.py`）、Isaac 控制（`touch_toy.py` 入口 + `sim_runtime.py`/`arm.py`/`planning.py`/`case_runner.py`）、离线自检与用例 |
| `inputs/`、`answers/` | 官方示例输入与程序生成的答案，一一对应 |
| `assets/` | 三份终端日志 |
| `video/` | 演示视频：`q1_rviz.mp4`（题目一 RViz）、`q3_touch.mp4`（题目三 Isaac Sim 触碰）、`q2_rviz.mp4`（题目二可视化，可选，一并提供）；均为 MP4/MPEG-4，单个不超过 30 MB |
| `docs/` | 题三接触证据图；`PROJECT.md` 为题二与题三的设计说明 |

## 已知限制与运行注意事项

- 题三只做“触碰”，不做抓取搬运；魔方、恐龙、软玩具都停在原位附近。
- 题三的证据图里，软玩具一格被腕部相机挡住手指，不如另两格直观；判据以日志数值为准。
- README 中题三的帧号只对 `video/q3_touch.mp4` 这一份视频有效，重跑后时序会变，需重新核验。
- 宽抓方案尚未做图形模式对照；判据为阈值，headless 与 GUI 的小幅差异不影响结论。
- 官方只给 `demo_zero` 一组金标，非零姿态靠单调性抽查与几何自检。
- 底盘锁定是“每个物理步写回根位姿”的近似，不是物理固定关节；实测位移 0.0000 米。
- Isaac Sim 对显存敏感，8 GB 是官方下限；运行前关掉其它占显存的程序。

## 原创内容与引用来源

- 机器人模型 `urdf/bajie_robot.urdf.xacro` 与场景 `sim/scene/abr_scene.usd` 来自赛事官方仓库，只读、未修改；题二内置副本只含 FK 需要的 xacro 与标定文件，不含 mesh。
- 第三方依赖：ROS 2 Humble、Isaac Sim 4.5、`numpy`、`xacro`、`opencv-python`、`pytest`，版本见上文环境表。
- 题一驱动、题二正运动学、题三规划与控制、三题的全部验证脚本均为本作品自行实现，未拷贝外部代码或答案；`answers/` 全部由程序运行生成。

