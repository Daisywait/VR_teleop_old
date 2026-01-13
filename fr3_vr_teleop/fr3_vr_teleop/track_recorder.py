#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import numpy as np
import pandas as pd
import time
import tf2_ros
from tf2_ros import TransformException
import threading

from geometry_msgs.msg import TransformStamped
from vive_ros2.msg import VRControllerData
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped

class TrackRecorder(Node):
    def __init__(self):
        super().__init__('track_recorder')
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 数据存储
        self.data = {
            'timestamp': [],
            # VR数据（右手）
            'vr_x': [], 'vr_y': [], 'vr_z': [],
            'vr_qx': [], 'vr_qy': [], 'vr_qz': [], 'vr_qw': [],
            # 机械臂末端（TF）
            'ee_x': [], 'ee_y': [], 'ee_z': [],
            'ee_qx': [], 'ee_qy': [], 'ee_qz': [], 'ee_qw': [],
            # 关节状态（只记录前7个关节）
            'j1': [], 'j2': [], 'j3': [], 'j4': [], 'j5': [], 'j6': [], 'j7': [],
            # 控制指令
            'cmd_vx': [], 'cmd_vy': [], 'cmd_vz': [],
            'cmd_wx': [], 'cmd_wy': [], 'cmd_wz': [],
            # 触发状态
            'trigger': []
        }
        
        # 订阅
        qos = QoSProfile(depth=100)
        
        # VR数据
        self.vr_sub = self.create_subscription(
            VRControllerData, '/controller_data', self.vr_callback, qos)
        
        # 关节状态
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, qos)
        
        # 控制指令
        self.cmd_sub = self.create_subscription(
            TwistStamped, '/moveit_servo/delta_twist_cmds', self.cmd_callback, qos)
        
        # 记录控制
        self.recording = False
        self.start_time = None
        self.record_duration = 30.0  # 记录30秒
        
        # 关节状态缓存
        self.current_joints = [0.0] * 7
        
        self.get_logger().info("✅ 轨迹记录器准备就绪")
        self.get_logger().info("5秒后自动开始记录30秒...")

    def vr_callback(self, msg):
        if not self.recording:
            return
            
        current_time = time.time()
        elapsed = current_time - self.start_time
        
        if elapsed > self.record_duration:
            self.stop_recording()
            return
        
        # 记录时间戳
        self.data['timestamp'].append(elapsed)
        
        # 记录VR数据
        self.data['vr_x'].append(msg.rel_pose.transform.translation.x)
        self.data['vr_y'].append(msg.rel_pose.transform.translation.y)
        self.data['vr_z'].append(msg.rel_pose.transform.translation.z)
        self.data['vr_qx'].append(msg.rel_pose.transform.rotation.x)
        self.data['vr_qy'].append(msg.rel_pose.transform.rotation.y)
        self.data['vr_qz'].append(msg.rel_pose.transform.rotation.z)
        self.data['vr_qw'].append(msg.rel_pose.transform.rotation.w)
        
        # 触发状态
        self.data['trigger'].append(int(msg.trigger_button))
        
        # 记录末端执行器位姿
        self.record_ee_pose()
        
        # 记录关节状态
        self.data['j1'].append(self.current_joints[0])
        self.data['j2'].append(self.current_joints[1])
        self.data['j3'].append(self.current_joints[2])
        self.data['j4'].append(self.current_joints[3])
        self.data['j5'].append(self.current_joints[4])
        self.data['j6'].append(self.current_joints[5])
        self.data['j7'].append(self.current_joints[6])
        
        # 定期显示进度
        if int(elapsed) % 5 == 0 and int(elapsed) > 0:
            if int(elapsed) % 5 == 0:  # 每5秒显示一次
                self.get_logger().info(f"记录中... {elapsed:.1f}/{self.record_duration}秒")

    def record_ee_pose(self):
        try:
            # 从TF获取末端执行器位姿
            trans = self.tf_buffer.lookup_transform(
                'fr3_link0',  # 基坐标系
                'robotiq_85_base_link',  # 末端执行器坐标系
                rclpy.time.Time()
            )
            
            self.data['ee_x'].append(trans.transform.translation.x)
            self.data['ee_y'].append(trans.transform.translation.y)
            self.data['ee_z'].append(trans.transform.translation.z)
            self.data['ee_qx'].append(trans.transform.rotation.x)
            self.data['ee_qy'].append(trans.transform.rotation.y)
            self.data['ee_qz'].append(trans.transform.rotation.z)
            self.data['ee_qw'].append(trans.transform.rotation.w)
            
        except (TransformException, AttributeError) as e:
            # TF查找失败，填充NaN
            self.data['ee_x'].append(np.nan)
            self.data['ee_y'].append(np.nan)
            self.data['ee_z'].append(np.nan)
            self.data['ee_qx'].append(np.nan)
            self.data['ee_qy'].append(np.nan)
            self.data['ee_qz'].append(np.nan)
            self.data['ee_qw'].append(np.nan)

    def joint_callback(self, msg):
        # 记录关节角度（假设前7个是Franka关节）
        if len(msg.position) >= 7:
            self.current_joints = list(msg.position[:7])

    def cmd_callback(self, msg):
        if not self.recording:
            return
            
        self.data['cmd_vx'].append(msg.twist.linear.x)
        self.data['cmd_vy'].append(msg.twist.linear.y)
        self.data['cmd_vz'].append(msg.twist.linear.z)
        self.data['cmd_wx'].append(msg.twist.angular.x)
        self.data['cmd_wy'].append(msg.twist.angular.y)
        self.data['cmd_wz'].append(msg.twist.angular.z)

    def start_recording(self):
        self.recording = True
        self.start_time = time.time()
        self.get_logger().info(f"🎬 开始记录轨迹，持续时间: {self.record_duration}秒")
        
        # 清空旧数据
        for key in self.data.keys():
            self.data[key] = []

    def stop_recording(self):
        if not self.recording:
            return
            
        self.recording = False
        
        # 保存数据
        self.save_data()
        self.get_logger().info("💾 数据保存完成")
        
        # 退出程序
        self.get_logger().info("🛑 记录完成，程序退出")
        threading.Thread(target=self.delayed_shutdown, args=(2,)).start()

    def delayed_shutdown(self, delay):
        time.sleep(delay)
        self.destroy_node()
        rclpy.shutdown()

    def save_data(self):
        # 确保所有列表长度相同
        max_len = max(len(self.data['timestamp']), 1)
        
        # 填充缺失的数据
        for key in self.data.keys():
            if len(self.data[key]) < max_len:
                # 填充NaN到相同长度
                self.data[key].extend([np.nan] * (max_len - len(self.data[key])))
            elif len(self.data[key]) > max_len:
                # 截断多余数据
                self.data[key] = self.data[key][:max_len]
        
        # 创建DataFrame并保存
        df = pd.DataFrame(self.data)
        
        # 保存为CSV
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"track_data_{timestamp}.csv"
        df.to_csv(filename, index=False)
        
        # 生成基本统计信息
        stats = f"""
轨迹记录统计
============
文件: {filename}
记录时长: {self.record_duration}秒
数据点数: {len(df)}
有效数据点: {df['vr_x'].count()}

VR位移范围:
  X: [{df['vr_x'].min():.3f}, {df['vr_x'].max():.3f}] m
  Y: [{df['vr_y'].min():.3f}, {df['vr_y'].max():.3f}] m  
  Z: [{df['vr_z'].min():.3f}, {df['vr_z'].max():.3f}] m

末端位移范围:
  X: [{df['ee_x'].min():.3f}, {df['ee_x'].max():.3f}] m
  Y: [{df['ee_y'].min():.3f}, {df['ee_y'].max():.3f}] m
  Z: [{df['ee_z'].min():.3f}, {df['ee_z'].max():.3f}] m
        """
        
        print(stats)
        
        # 保存统计信息
        with open(f"track_stats_{timestamp}.txt", 'w') as f:
            f.write(stats)
        
        self.get_logger().info(f"数据已保存: {filename}")

def main():
    rclpy.init()
    node = TrackRecorder()
    
    print("\n" + "="*60)
    print("轨迹记录器")
    print("="*60)
    print("5秒后开始自动记录...")
    print("按 Ctrl+C 停止记录")
    print("="*60)
    
    # 使用线程定时器而不是ROS定时器
    import threading
    
    # 5秒后开始记录
    start_timer = threading.Timer(5.0, node.start_recording)
    start_timer.start()
    
    # 35秒后停止记录（30秒记录 + 5秒等待）
    stop_timer = threading.Timer(35.0, node.stop_recording)
    stop_timer.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n用户中断，停止记录...")
        node.stop_recording()
        # 取消定时器
        start_timer.cancel()
        stop_timer.cancel()
    finally:
        # 确保定时器被取消
        try:
            start_timer.cancel()
            stop_timer.cancel()
        except:
            pass

if __name__ == '__main__':
    main()