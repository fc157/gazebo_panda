# Gazebo Panda - Franka Emika Panda Robot Simulation

基于 ROS 2 的 Franka Emika Panda 机械臂 + 夹爪 Gazebo 仿真项目，集成 MoveIt 2 运动规划、手眼相机 ArUco 码检测与定位控制。

---

## 模块结构

```
gazebo_panda/
├── panda/                   # 机器人模型与 Gazebo 仿真核心
├── panda_moveit_config/     # MoveIt 2 运动规划配置
├── moveit_control/          # MoveIt 控制 GUI (Qt5)
└── camera_perception/       # 相机感知：ArUco 码检测与移动控制
```

---

## 各模块作用

### 1. panda — 机器人仿真核心

**描述：** Franka Emika Panda 机械臂 + 夹爪的 Gazebo 仿真包，包含 URDF/XACRO 模型描述、3D 网格文件、控制器配置和启动文件。

**主要文件：**
- `urdf/panda_arm_hand.urdf.xacro` — 完整的机械臂 + 夹爪 URDF 描述
- `urdf/hand.xacro` — 夹爪 URDF
- `urdf/camera.xacro` — 手眼相机 URDF
- `urdf/panda.gazebo.xacro` — Gazebo 仿真参数
- `urdf/ros_control.xacro` — ros2_control 配置
- `config/ros2_controllers.yaml` — 控制器配置（joint_state_broadcaster, panda_arm_controller, panda_gripper_controller）
- `config/panda.rviz` — RViz 可视化配置
- `launch/gazebo.launch.py` — 启动 Gazebo + 机器人 + 控制器 + RViz
- `launch/panda_controller.launch.py` — 启动 panda 控制节点
- `src/panda_node.cpp` — 控制节点，发送关节轨迹目标到 `/panda/panda_joint_trajectory_controller/follow_joint_trajectory`
- `meshes/` — 3D 网格文件（visual/collision）
- `models/` — Gazebo 模型（如 ArUco 码模型）
- `worlds/` — Gazebo 世界文件（empty, aruco, calibration）

**启动命令：**

```bash
# 启动 Gazebo + Panda 机器人 + 控制器（含 RViz）
ros2 launch panda gazebo.launch.py

# 使用自定义世界文件
ros2 launch panda gazebo.launch.py world:=$(ros2 pkg prefix panda)/share/panda/worlds/aruco.world

# 启动 Panda 控制节点（发送预设关节轨迹）
ros2 launch panda panda_controller.launch.py

# 单独启动 RViz
rviz2 -d $(ros2 pkg prefix panda)/share/panda/config/panda.rviz
```

**控制器加载命令（如需手动加载）：**

```bash
# 依次加载控制器（需在 Gazebo 启动后执行）
ros2 control load_controller --set-state start joint_state_broadcaster
ros2 control load_controller --set-state start panda_arm_controller
ros2 control load_controller --set-state start panda_gripper_controller
```

---

### 2. panda_moveit_config — MoveIt 2 运动规划配置

**描述：** Franka Emika Panda 的 MoveIt 2 配置包，提供 OMPL 运动规划器、运动学求解器（IKFast/TRAC-IK）、碰撞检测和环境感知配置。

**主要配置：**
- `config/kinematics.yaml` — 运动学求解器配置
- `config/joint_limits.yaml` — 关节限位
- `config/ompl_planning.yaml` — OMPL 规划器参数
- `config/panda.srdf` — 语义机器人描述（SRDF），定义规划组、姿态、碰撞对等
- `config/moveit_controllers.yaml` — MoveIt 控制器映射
- `config/sensors_3d.yaml` — 3D 传感器配置

**启动命令：**

```bash
# 完整启动：Gazebo + Robot + 控制器 + MoveIt + RViz（推荐）
ros2 launch panda_moveit_config demo.launch.py

# 仅启动 MoveIt + RViz（无需 Gazebo，使用 fake 硬件）
ros2 launch panda_moveit_config demo_fake.launch.py

# 仅启动 MoveIt RViz（需先运行 demo.launch.py 或单独启动 move_group）
ros2 launch panda_moveit_config moveit_rviz.launch.py

# 测试 MoveIt 运动规划（需先启动 demo.launch.py）
ros2 launch panda_moveit_config moveit_test.launch.py
```

**⚠️ 时序说明（demo.launch.py）：**

| 时间     | 动作                          |
|----------|-------------------------------|
| t=0.0s   | Gazebo server/client, RSP, spawn robot |
| t=5.0s   | joint_state_broadcaster       |
| t=7.0s   | panda_arm_controller          |
| t=9.0s   | panda_gripper_controller      |
| t=10.0s  | move_group (MoveIt)           |
| t=12.0s  | RViz (with MoveIt plugin)     |

---

### 3. moveit_control — MoveIt 控制 GUI

**描述：** 基于 Qt5 的 C++ 图形界面，通过 MoveIt 2 的 move_group 接口控制 Panda 机械臂运动。需在 move_group 运行后启动。

**主要文件：**
- `src/moveit_control_gui.cpp` — GUI 主程序，集成 MoveIt 运动规划
- `include/moveit_control_gui/main_window.hpp` — 主窗口头文件
- `launch/moveit_control.launch.py` — 启动文件

**启动命令：**

```bash
# 终端1：启动完整仿真 + MoveIt
ros2 launch panda_moveit_config demo.launch.py

# 终端2：待 demo 完全启动后，启动控制 GUI
ros2 launch moveit_control moveit_control.launch.py
```

---

### 4. camera_perception — 相机感知模块

**描述：** 手眼相机的视觉感知包，包含 ArUco 码实时检测与 ArUco 码位置控制 GUI。

#### 4.1 aruco_detector — ArUco 码检测

**描述：** 实时检测 ArUco 标识码，订阅手眼相机图像，发布检测到的 ArUco 码位姿和标注后的调试图像。

**订阅：**
- `/hand_camera/image_raw` (`sensor_msgs/Image`) — 相机 RGB 图像

**发布：**
- `/detected_aruco_single` (`geometry_msgs/PoseStamped`) — 检测到的 ArUco 码位姿（参考系 `panda_camera_optical_frame`）
- `/detected_aruco_image` (`sensor_msgs/Image`) — 标注了检测结果的调试图像

**启动命令：**

```bash
# 单独启动 ArUco 码检测（需先启动 Gazebo + 相机）
ros2 launch camera_perception aruco_detector.launch.py
```

#### 4.2 aruco_mover — ArUco 码位置控制 GUI

**描述：** 基于 tkinter 的图形界面，通过 Gazebo 的 `/gazebo/set_entity_state` 服务动态移动场景中的 `aruco_marker_1` 模型。支持平滑插值、循环往返运动。

**功能：**
- 通过滑块设置目标位置（X, Y, Z）和姿态（Roll, Pitch, Yaw）
- 可调节运动持续时间（0.1~10s）
- 单次移动 / 循环往返模式
- 实时显示当前位置和移动进度

**启动命令：**

```bash
# 启动 ArUco 码位置控制 GUI（需先启动 Gazebo + 包含 aruco_marker_1 的世界）
ros2 launch camera_perception aruco_mover.launch.py

# 自定义参数
ros2 launch camera_perception aruco_mover.launch.py update_rate:=5.0 reference_frame:=world
```

#### 4.3 aruco_demo — ArUco 视觉伺服完整演示

**描述：** 一键启动 ArUco 码视觉伺服完整演示，包含：Gazebo（ArUco 世界）→ 机器人 → 控制器 → ArUco 检测 → RViz。

**启动命令：**

```bash
# 启动完整 ArUco 演示（推荐）
ros2 launch camera_perception aruco_demo.launch.py

# 不使用 RViz
ros2 launch camera_perception aruco_demo.launch.py use_rviz:=false

# 终端2：启动 MoveIt 控制 GUI（可选）
ros2 launch moveit_control moveit_control.launch.py
```

---

## 常用 ROS 2 控制指令

### 夹爪控制

```bash
# 夹爪打开（0.035 表示打开，数值越小表示夹爪闭合越紧）
ros2 action send_goal /panda_gripper_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: ['panda_finger_joint1','panda_finger_joint2'], \
  points: [{positions: [0.035,0.035], velocities: [0.0,0.0], time_from_start: {sec: 1, nanosec: 0}}]}}"

# 夹爪闭合（0.001 表示完全闭合）
ros2 action send_goal /panda_gripper_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: ['panda_finger_joint1','panda_finger_joint2'], \
  points: [{positions: [0.001,0.001], velocities: [0.0,0.0], time_from_start: {sec: 1, nanosec: 0}}]}}"
```

### 机械臂控制

```bash
# 通过关节轨迹控制器发送目标
ros2 action send_goal /panda_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: ['panda_joint1','panda_joint2','panda_joint3','panda_joint4', \
  'panda_joint5','panda_joint6','panda_joint7'], \
  points: [{positions: [0.0, -0.5, 0.0, -1.5, 0.0, 1.0, 0.5], \
  velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 3, nanosec: 0}}]}}"
```

### 查看关节状态

```bash
# 查看关节状态话题
ros2 topic echo /joint_states

# 列出所有活跃的控制器
ros2 control list_controllers
```

---

## 开发记录

- `2026.6.5` — MoveIt 手臂配置完成，camera 位置设置完成
- `2026.6.9` — 调整 camera 视角方向
- `2026.6.10` — 代码中控制夹爪可实现
- `2026.6.11` — RViz 和 ros2 指令均可控制夹爪