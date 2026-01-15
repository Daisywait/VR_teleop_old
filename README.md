# Vive ROS2 遥操作系统

这是一个基于 ROS2 的 HTC Vive VR 控制器遥操作系统，用于通过 VR 控制器实时控制机器人（Franka Emika FR3 机械臂）。该系统提供了完整的 VR 控制器数据获取、处理和机器人控制功能。

## 📋 项目概述

本项目包含两个主要的功能包：

1. **`vive_ros2`** - VR 控制器与 ROS2 的接口包
2. **`fr3_vr_teleop`** - Franka FR3 机器人的 VR 遥操作包

### 核心功能

- ✅ **VR 控制器数据获取**：通过 OpenVR SDK 获取 HTC Vive 控制器的位姿、按钮和摇杆数据
- ✅ **实时位姿发布**：发布控制器的绝对位姿和相对位姿到 ROS2 话题
- ✅ **机器人遥操作**：将 VR 控制器的运动映射到机器人末端执行器（通过 TF 获取 `robotiq_85_base_link` 坐标系位姿，使用 MoveIt Servo 进行速度控制）
- ✅ **夹爪控制**：通过 VR 控制器手柄控制机器人夹爪
- ✅ **双控制器支持**：支持同时连接两个控制器，通过 OpenVR SDK 的控制器角色识别功能区分并独立处理

## 🏗️ 系统架构

```
┌─────────────────┐
│  HTC Vive 控制器 │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  OpenVR SDK     │
└────────┬────────┘
         │
         ▼
┌─────────────────┐      Socket      ┌─────────────────┐
│  vive_input     │◄─────────────────►│  vive_node      │
│  (VR 数据采集)   │   (TCP/IP)       │  (ROS2 节点)     │
└─────────────────┘                  └────────┬────────┘
                                              │
                                              ▼
                                    ┌─────────────────┐
                                    │  /controller_data│
                                    │  (ROS2 话题)     │
                                    └────────┬────────┘
                                             │
                                             ▼
                                    ┌─────────────────┐
                                    │ fr3_vr_teleop   │
                                    │ (遥操作节点)     │
                                    └────────┬────────┘
                                             │
                                             ▼
                                    ┌─────────────────┐
                                    │ MoveIt Servo    │
                                    │ (速度控制)       │
                                    └────────┬────────┘
                                             │
                                             ▼
                                    ┌─────────────────┐
                                    │  Franka FR3     │
                                    │  机械臂          │
                                    └─────────────────┘
```

### ROS2 话题发布订阅关系

```
┌─────────────────────────────────────────────────────────────┐
│                      vive_node                               │
│  ┌──────────────────────────────────────────────────────┐   │
│  │  发布话题：                                           │   │
│  │  • /controller_data (VRControllerData)              │   │
│  │  • /vive_pose_abs (TransformStamped)                 │   │
│  │  • /vive_pose_rel (TransformStamped)                 │   │
│  │  • /left_vr/vive_pose_abs, /left_vr/vive_pose_rel   │   │
│  │  • /right_vr/vive_pose_abs, /right_vr/vive_pose_rel │   │
│  │  • TF 树发布：发布控制器在 world 坐标系中的位姿   │   │
│  │    （主要用于 RViz 可视化，实际控制不依赖 TF）    │   │
│  │    便于调试：可以查看控制器相对于 world 的位姿    │   │
│  │    - world → left_vr/vive_pose_abs                 │   │
│  │    - world → right_vr/vive_pose_abs                │   │
│  │    - world → left_vr/vive_pose_rel                 │   │
│  │    - world → right_vr/vive_pose_rel                │   │
│  └──────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
                            │
                            │ 订阅
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                  fr3_vr_teleop_node                         │
│  ┌──────────────────────────────────────────────────────┐   │
│  │  订阅话题：                                           │   │
│  │  • /controller_data (VRControllerData)              │   │
│  │     └─ 使用 rel_pose 字段进行遥操作控制              │   │
│  │                                                       │   │
│  │  订阅 TF：                                            │   │
│  │  • fr3_link0 → robotiq_85_base_link                  │   │
│  │     └─ 获取末端执行器相对于基坐标系 fr3_link0 的位姿│   │
│  │        （通过 tf_buffer.lookup_transform() 查询）     │   │
│  │                                                       │   │
│  │  发布话题：                                           │   │
│  │  • /moveit_servo/delta_twist_cmds (TwistStamped)    │   │
│  │     └─ 速度控制指令                                  │   │
│  │                                                       │   │
│  │  Action 客户端：                                     │   │
│  │  • /robotiq_gripper_controller/gripper_cmd          │   │
│  │     └─ 夹爪控制                                      │   │
│  └──────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
                            │
                            │ 发布
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                    MoveIt Servo                             │
│  ┌──────────────────────────────────────────────────────┐   │
│  │  订阅话题：                                           │   │
│  │  • /moveit_servo/delta_twist_cmds                    │   │
│  │     └─ 转换为关节速度指令                            │   │
│  └──────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

## 📦 功能包说明

### 1. vive_ros2

**功能描述**：提供 HTC Vive VR 控制器与 ROS2 之间的接口。

**主要特性**：
- **通过 OpenVR SDK 获取 VR 控制器数据**：使用 `vr::IVRSystem` 接口获取控制器位姿和按钮状态
- **使用 Socket 通信解决 OpenVR 与 ROS2 的兼容性问题**：`vive_input` 程序在独立线程中启动 TCP 服务器，监听端口 12345，等待 `vive_node` 连接后通过 Socket 传输数据
- **发布控制器的绝对位姿和相对位姿**：发布到 `/vive_pose_abs` 和 `/vive_pose_rel` 话题
- **坐标系统转换**：通过 `transformVRToROS()` 函数将 VR Y-up 坐标系转换为 ROS Z-up 坐标系，仅应用于发布到 TF 树和 `/vive_pose_abs`、`/vive_pose_rel` 话题的数据，而 `/controller_data` 话题中的位姿数据保持 VR 原始坐标系

**关键节点**：
- `vive_input`：VR 数据采集程序，通过 Socket 发送数据
- `vive_node`：ROS2 节点，接收 Socket 数据并发布到 ROS2 话题

**发布的话题**：
- `/controller_data`：完整的控制器数据（`vive_ros2/msg/VRControllerData`），包含 `abs_pose` 和 `rel_pose` 字段
  - `rel_pose`：相对位姿，表示相对于按下 Trigger 键时的初始位姿的变化，用于遥操作控制（`fr3_vr_teleop_node.py` 中使用）
  - `abs_pose`：绝对位姿，表示控制器在 VR 世界坐标系中的位姿，当前代码中未直接使用，可用于可视化或调试
- `/vive_pose_abs`：绝对位姿（`geometry_msgs/TransformStamped`），目前未被代码订阅，可用于可视化或调试
- `/vive_pose_rel`：相对位姿（`geometry_msgs/TransformStamped`），目前未被代码订阅，可用于可视化或调试
- `/left_vr/vive_pose_abs`、`/left_vr/vive_pose_rel`：左角色控制器位姿
- `/right_vr/vive_pose_abs`、`/right_vr/vive_pose_rel`：右角色控制器位姿

**消息类型**：
- `VRControllerData.msg`：包含按钮状态、摇杆数据、触发器和位姿信息

### 2. fr3_vr_teleop

**功能描述**：实现 Franka Emika FR3 机器人的 VR 遥操作控制。

**主要特性**：
- **位姿映射**：将 VR 控制器的相对运动映射到机器人末端执行器
- **速度控制**：使用 MoveIt Servo 进行平滑的速度控制
- **夹爪控制**：通过控制器摇杆输入连续控制夹爪开合
- **闭环控制**：基于当前末端位姿和目标位姿的误差进行 P 控制

**控制方式**：
- **运动控制**：按下 Trigger 键开始控制，松开停止
- **夹爪控制**：通过控制器摇杆上下滑动控制夹爪开合
- **锚点机制**：按下 Trigger 时建立当前位姿为锚点，后续运动相对于锚点

**关键节点**：
- `fr3_vr_teleop_node`：主遥操作节点

**订阅的话题**：
- `/controller_data`：VR 控制器数据

**发布的话题**：
- `/moveit_servo/delta_twist_cmds`：速度控制指令（`geometry_msgs/TwistStamped`）

**Action 客户端**：
- `/robotiq_gripper_controller/gripper_cmd`：夹爪控制（`control_msgs/GripperCommand`）

**可配置参数**：
- `linear_scale`：位置控制增益（默认：2.1）
- `angular_scale`：姿态控制增益（默认：0.4）
- `v_max`：最大线速度（默认：0.15 m/s）
- `w_max`：最大角速度（默认：1.0 rad/s）
- `smoothing_factor`：平滑因子（默认：0.3）
- `deadzone_linear`：线性死区（默认：0.002 m）
- `deadzone_angular`：角度死区（默认：0.03 rad）
- `gripper_speed`：夹爪速度（默认：0.8 全行程/秒）
- `gripper_axis`：夹爪控制轴（默认：`trackpad_y`）

## 🚀 运行与使用

### 运行系统

#### 1. 启动 ALVR和SteamVR
```bash
#待补充
```

#### 2. 启动 VR 数据采集节点

```bash
# 终端 1：启动 VR 输入程序
ros2 run vive_ros2 vive_input

# 终端 2：启动 VR ROS2 节点
ros2 run vive_ros2 vive_node 100  # 100 Hz 发布频率
```

#### 3. 启动机器人遥操作

```bash
# 终端 3：启动遥操作节点
ros2 run fr3_vr_teleop fr3_vr_teleop_node
```

### 使用方法

1. **开始控制**：
   - 按下 VR 控制器的 Trigger 键开始控制机器人
   - 移动控制器，机器人末端执行器会跟随运动

2. **停止控制**：
   - 松开 Trigger 键，机器人会平滑停止

3. **控制夹爪**：
   - 上下滑动摇杆控制夹爪开合

4. **重置锚点**：
   - 按下 Menu 键可以重置相对位姿的锚点

## 📊 数据流

### VR 控制器数据

VR 控制器数据通过以下流程传递：

1. **硬件层**：HTC Vive 控制器 → OpenVR SDK
2. **采集层**：`vive_input` 程序读取 OpenVR 数据
3. **通信层**：通过 TCP Socket (127.0.0.1:12345) 传输 JSON 数据
4. **ROS2 层**：`vive_node` 接收数据并发布到 ROS2 话题
5. **应用层**：`fr3_vr_teleop` 订阅数据并控制机器人

### 控制指令流

1. VR 控制器运动 → 相对位姿变化
2. 相对位姿 → 期望末端位姿（基于锚点）
3. 期望位姿 - 当前位姿 → 位置和姿态误差
4. 误差 × 增益 → 速度指令（Twist）
5. 速度指令 → MoveIt Servo → 机器人关节速度

## 🔧 技术细节

### 坐标系统

- **VR 坐标系**：Y-up（Y 轴向上）
- **ROS 坐标系**：Z-up（Z 轴向上）
- 系统中有两个坐标转换阶段

#### 转换 1：vive_node.cpp 中的坐标转换（VR Y-up → ROS Z-up）

**应用范围**：仅应用于发布到 TF 树和 `/vive_pose_abs`、`/vive_pose_rel` 话题的数据

**注意**：`/controller_data` 话题中的位姿数据保持 VR 原始坐标系，不进行此转换。

**坐标转换示意图**：
```
VR 坐标系 (Y-up)                    ROS 坐标系 (Z-up)
┌─────────────┐                    ┌─────────────┐
│      Y↑     │                    │      Z↑     │
│      │      │                    │      │      │
│      │      │                    │      │      │
│      └──X→  │                    │      └──X→  │
│     ╱       │                    │     ╱       │
│    ╱        │                    │    ╱        │
│   Z⊙        │                    │   Y⊙        │
└─────────────┘                    └─────────────┘
X: 右 → 前                         X: 前
Y: 上 → 上                         Y: 左
Z: 后 → 上                         Z: 上

坐标轴映射关系：
VR → ROS
─────────────────────────────
位置 (Translation):
  VR.z (后)  → ROS.x (前)   [取反]
  VR.x (右)  → ROS.y (左)   [取反]
  VR.y (上)  → ROS.z (上)   [直接对应]

旋转 (Quaternion):
  VR.qz      → ROS.qx       [取反]
  VR.qx      → ROS.qy       [取反]
  VR.qy      → ROS.qz       [直接对应]
  VR.qw      → ROS.qw       [直接对应]
```

**实现代码**（`vive_node.cpp` 的 `transformVRToROS()` 函数）：
```cpp
void transformVRToROS(const VRControllerData& vrData, 
                      geometry_msgs::msg::TransformStamped& transform) {
    // 位置转换：VR (Y-up) → ROS (Z-up)
    transform.transform.translation.x = -vrData.pose_z;  // VR.z(后) → ROS.x(前)
    transform.transform.translation.y = -vrData.pose_x;  // VR.x(右) → ROS.y(左)
    transform.transform.translation.z = vrData.pose_y;   // VR.y(上) → ROS.z(上)
    
    // 旋转转换：四元数轴映射
    transform.transform.rotation.x = -vrData.pose_qz;
    transform.transform.rotation.y = -vrData.pose_qx;
    transform.transform.rotation.z = vrData.pose_qy;
    transform.transform.rotation.w = vrData.pose_qw;
}
```

#### 转换 2：fr3_vr_teleop_node.py 中的坐标转换（VR 坐标系 → 机器人规划坐标系）

**应用范围**：用于遥操作控制，将 VR 控制器的相对位姿转换为机器人规划坐标系（`fr3_link0`）中的位姿

**输入数据**：来自 `/controller_data` 消息的 `rel_pose` 字段（保持 VR 原始坐标系）

**坐标轴映射**：
```
VR 坐标系 → 机器人规划坐标系 (fr3_link0, ROS Z-up)
─────────────────────────────────────────────────────
位置映射：
  robot.x ← VR.z (后→前)
  robot.y ← -VR.x (右→左)
  robot.z ← VR.y (上→上)

旋转映射（通过旋转向量）：
  robot.x ← -VR.z
  robot.y ← VR.x
  robot.z ← VR.y
```

**实现代码**（`fr3_vr_teleop_node.py` 的 `process_motion_pose_servo()` 函数）：
```python
# === 计算 VR 相对变化 ===
vr_anchor_pi, vr_anchor_qi = pose_inverse(self.vr_anchor_p, self.vr_anchor_q)
dvr_p, dvr_q = pose_compose(vr_anchor_pi, vr_anchor_qi, vr_p_now, vr_q_now)

# === 坐标轴映射：VR → Robot Planning Frame ===
# VR: [x=右, y=上, z=后]  -> robot planning_frame
dp_robot = np.array([
    dvr_p[2],    # robot.x ← VR.z (后→前)
    -dvr_p[0],   # robot.y ← -VR.x (右→左)
    dvr_p[1],    # robot.z ← VR.y (上→上)
], dtype=float)

# === 旋转映射：VR → Robot Planning Frame ===
rotvec_raw = R.from_quat(dvr_q).as_rotvec()  # [rx, ry, rz] in VR frame
rotvec_robot = np.array([
    -rotvec_raw[2],  # robot.x ← -VR.z
    rotvec_raw[0],   # robot.y ← VR.x
    rotvec_raw[1],   # robot.z ← VR.y
], dtype=float)

# 将旋转向量转回四元数
dR_robot = R.from_rotvec(rotvec_robot)
dq_robot = quat_normalize(dR_robot.as_quat())

# === 应用到机器人末端执行器 ===
p_des, q_des = pose_compose(self.ee_anchor_p, self.ee_anchor_q, dp_robot, dq_robot)
```

**坐标转换流程**：
1. 从 `/controller_data` 消息的 `rel_pose` 字段获取 VR 控制器的相对位姿（VR 原始坐标系）
2. 计算 VR 控制器相对于锚点的变化量：`ΔT_vr = T_vr_anchor^-1 ⊗ T_vr_now`
3. 通过坐标轴映射将 VR 坐标系转换为机器人规划坐标系（ROS Z-up）
4. 将转换后的位姿变化应用到机器人末端执行器锚点：`T_des = T_ee_anchor ⊗ ΔT_robot`
5. 计算当前位姿与期望位姿的误差，生成速度控制指令

**说明**：机器人规划坐标系（`fr3_link0`）遵循 ROS 标准，为 Z-up 坐标系，与 ROS 坐标系一致。

### 位姿计算

- **绝对位姿**：控制器在 VR 世界坐标系中的位姿
- **相对位姿**：相对于按下 Trigger 时的初始位姿的变化
- 使用四元数进行旋转计算

### 控制算法

- **P 控制**：基于位置和姿态误差的比例控制
- **速度限制**：限制最大线速度和角速度
- **平滑滤波**：使用低通滤波器平滑速度指令
- **死区处理**：小误差时不输出指令，避免抖动






