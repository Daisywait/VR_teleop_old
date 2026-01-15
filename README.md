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
│  │     └─ 获取当前末端执行器位姿                        │   │
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
- 系统自动进行坐标转换

### 位姿计算

- **绝对位姿**：控制器在 VR 世界坐标系中的位姿
- **相对位姿**：相对于按下 Trigger 时的初始位姿的变化
- 使用四元数进行旋转计算

### 控制算法

- **P 控制**：基于位置和姿态误差的比例控制
- **速度限制**：限制最大线速度和角速度
- **平滑滤波**：使用低通滤波器平滑速度指令
- **死区处理**：小误差时不输出指令，避免抖动

## 📝 开发状态

### 已完成功能

- [x] VR 控制器数据获取
- [x] 绝对和相对位姿发布
- [x] 双控制器支持
- [x] 机器人遥操作控制
- [x] 夹爪连续控制
- [x] 坐标系统转换
- [x] 平滑控制和平滑停止


