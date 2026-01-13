#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import TwistStamped
from vive_ros2.msg import VRControllerData
import numpy as np

class FR3VRLinearDebug(Node):
    def __init__(self):
        super().__init__('fr3_vr_test')

        # 参数配置
        self.declare_parameter('planning_frame', 'fr3_link0')
        self.declare_parameter('linear_scale', 1.2)      # 稍微调大，让动作更明显
        self.declare_parameter('smoothing_factor', 0.5)  # 降低平滑，响应更直接
        self.declare_parameter('deadzone_linear', 0.01) # 设置极小死区 (0.5cm)

        self.is_controlling = False
        self.target_twist = TwistStamped()

        # 通信接口
        qos = QoSProfile(depth=10)
        self.controller_sub = self.create_subscription(
            VRControllerData, '/controller_data', self.vr_callback, qos)
        self.twist_pub = self.create_publisher(
            TwistStamped, '/moveit_servo/delta_twist_cmds', qos)

        # 50Hz 定时器发布指令
        self.timer_twist = self.create_timer(0.02, self.publish_twist)
        
        self.get_logger().info("🛠️ [调试模式] 启动：重点观察终端打印的数值映射")

    def vr_callback(self, msg):
        # 使用模拟量 trigger (0.0 ~ 1.0)
        # 只要扣下一点点 (0.1) 就开始运动
        if msg.trigger > 0.1:
            if not self.is_controlling:
                self.get_logger().info("🎮 触发按下 - 开始控制")
                self.is_controlling = True
            self.process_motion(msg)
        else:
            if self.is_controlling:
                self.get_logger().info("⏸️ 停止运动")
                self.is_controlling = False
            self.smooth_stop()

    def process_motion(self, msg):
        scale = self.get_parameter('linear_scale').value
        alpha = self.get_parameter('smoothing_factor').value
        dz = self.get_parameter('deadzone_linear').value

        # 1. VR 原始坐标 (保持不变)
        vx = msg.rel_pose.transform.translation.x  
        vy = msg.rel_pose.transform.translation.y  
        vz = msg.rel_pose.transform.translation.z  

  
        tx =  vz * scale  # 这里的 tx 对应机器人 X (前后)

        ty = vx * scale  # 这里的 ty 对应机器人 Y (左右)
        
        # 上下方向保持不变
        tz =  vy * scale  # 这里的 tz 对应机器人 Z (上下)
        # -----------------------------------------------------------

        # 3. 打印调试（让你看到映射后的结果）
        self.get_logger().info(
            f"物理[vx:{vx:+.3f}] -> 逻辑[X前:{tx:+.3f}] | "
            f"物理[vz:{vz:+.3f}] -> 逻辑[Y左:{ty:+.3f}]"
        )

        # 4. 应用死区
        dist = np.sqrt(tx**2 + ty**2 + tz**2)
        if dist < dz:
            tx, ty, tz = 0.0, 0.0, 0.0

        # 5. 滤波赋值 (保持不变)
        self.target_twist.twist.linear.x = self.target_twist.twist.linear.x * (1-alpha) + tx * alpha
        self.target_twist.twist.linear.y = self.target_twist.twist.linear.y * (1-alpha) + ty * alpha
        self.target_twist.twist.linear.z = self.target_twist.twist.linear.z * (1-alpha) + tz * alpha
        
        self.target_twist.twist.angular.x = 0.0
        self.target_twist.twist.angular.y = 0.0
        self.target_twist.twist.angular.z = 0.0
        
    def smooth_stop(self):
        # 快速归零
        self.target_twist.twist.linear.x = 0.0
        self.target_twist.twist.linear.y = 0.0
        self.target_twist.twist.linear.z = 0.0

    def publish_twist(self):
        self.target_twist.header.stamp = self.get_clock().now().to_msg()
        self.target_twist.header.frame_id = self.get_parameter('planning_frame').value
        self.twist_pub.publish(self.target_twist)

def main():
    rclpy.init()
    node = FR3VRLinearDebug()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()