#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, TransformStamped
from vive_ros2.msg import VRControllerData
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Vector3
import math

class FR3VRTeleop(Node):
    def __init__(self):
        super().__init__('fr3_vr_teleop')
        
        # 参数声明
        self.declare_parameter('linear_scale', 0.1)
        self.declare_parameter('angular_scale', 0.5)
        self.declare_parameter('deadzone_translation', 0.01)
        self.declare_parameter('deadzone_rotation', 0.05)
        self.declare_parameter('planning_frame', 'fr3_link0')
        self.declare_parameter('max_linear_velocity', 0.5)
        self.declare_parameter('max_angular_velocity', 1.0)
        
        # TF2 相关
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 状态变量
        self.is_controlling = False
        self.control_start_time = None
        self.last_abs_pose = None
        self.last_time = None
        
        # 只订阅 /controller_data 话题
        self.controller_data_sub = self.create_subscription(
            VRControllerData,
            '/controller_data',  # 唯一的数据源
            self.controller_data_callback,
            10
        )
        
        # 发布到 Servo 节点
        self.twist_pub = self.create_publisher(
            TwistStamped,
            '/moveit_servo/delta_twist_cmds',
            10
        )
        
        self.get_logger().info("FR3 VR遥操作节点已启动 - 从controller_data提取位姿数据")

    def controller_data_callback(self, msg):
        """
        controller_data 话题回调函数
        从这里提取右手控制器的abs_pose和rel_pose
        """
        try:
            # 条件1: 只处理右手控制器
            if msg.role != 0:
                return
            
            # 条件2: 只处理扳机按下的情况
            if not msg.trigger_button:
                # 扳机释放 - 退出控制模式
                if self.is_controlling:
                    self.is_controlling = False
                    self.publish_zero_velocity()
                    if self.control_start_time:
                        control_duration = (self.get_clock().now() - self.control_start_time).nanoseconds / 1e9
                        self.get_logger().info(f"⏹️ 扳机释放 - 退出控制模式 (持续时间: {control_duration:.2f}s)")
                    self.last_abs_pose = None
                return
            
            # 条件3: 检查数据完整性
            if not hasattr(msg, 'abs_pose') or not hasattr(msg, 'rel_pose'):
                self.get_logger().warn("controller_data消息缺少位姿数据")
                return
            
            # 所有条件满足，开始处理数据
            self.process_controller_data(msg)
            
        except Exception as e:
            self.get_logger().error(f"处理controller_data出错: {e}")

    def process_controller_data(self, msg):
        """
        处理控制器数据 - 从单个消息中提取所有需要的信息
        """
        # 进入控制模式
        if not self.is_controlling:
            self.is_controlling = True
            self.control_start_time = self.get_clock().now()
            self.last_abs_pose = None
            self.last_time = None
            self.get_logger().info("🎮 扳机按下 - 进入控制模式")
        
        # 从当前消息中提取位姿数据
        current_abs_pose = msg.abs_pose
        current_rel_pose = msg.rel_pose
        
        # 处理并发布控制命令
        self.process_and_publish_command(current_abs_pose, current_rel_pose)

    def process_and_publish_command(self, abs_pose, rel_pose):
        """
        处理并发布控制命令
        """
        try:
            # 获取控制参数
            linear_scale = self.get_parameter('linear_scale').value
            angular_scale = self.get_parameter('angular_scale').value
            deadzone_trans = self.get_parameter('deadzone_translation').value
            deadzone_rot = self.get_parameter('deadzone_rotation').value
            planning_frame = self.get_parameter('planning_frame').value
            
            # 坐标变换：将绝对位姿转换到规划坐标系
            transformed_abs_pose = self.transform_pose_to_planning_frame(abs_pose, planning_frame)
            
            if transformed_abs_pose:
                # 计算并发布速度命令
                twist_stamped = self.calculate_velocity_command(
                    transformed_abs_pose, rel_pose, linear_scale, angular_scale, 
                    deadzone_trans, deadzone_rot, planning_frame)
                
                if twist_stamped:
                    self.twist_pub.publish(twist_stamped)
                    
                    # 调试日志
                    self.get_logger().debug(
                        f"控制命令 - "
                        f"线速度: ({twist_stamped.twist.linear.x:.3f}, "
                        f"{twist_stamped.twist.linear.y:.3f}, "
                        f"{twist_stamped.twist.linear.z:.3f}) | "
                        f"角速度: ({twist_stamped.twist.angular.x:.3f}, "
                        f"{twist_stamped.twist.angular.y:.3f}, "
                        f"{twist_stamped.twist.angular.z:.3f})",
                        throttle_duration_sec=1.0
                    )
            
        except Exception as e:
            self.get_logger().error(f"处理发布命令出错: {e}")

    def calculate_velocity_command(self, abs_pose, rel_pose, linear_scale, angular_scale, 
                                 deadzone_trans, deadzone_rot, planning_frame):
        """
        计算速度命令
        使用绝对位姿进行坐标参考，使用相对位姿计算速度增量
        """
        current_time = self.get_clock().now()
        
        # 初始化或重置
        if self.last_abs_pose is None or self.last_time is None:
            self.last_abs_pose = abs_pose
            self.last_time = current_time
            return None
        
        # 计算时间增量
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return None
        
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = current_time.to_msg()
        twist_stamped.header.frame_id = planning_frame
        
        # 方法1: 使用相对位姿直接计算速度（推荐）
        # 因为rel_pose已经是相对于上一时刻的增量
        dx = rel_pose.transform.translation.x
        dy = rel_pose.transform.translation.y
        dz = rel_pose.transform.translation.z
        
        # 应用死区和缩放
        if abs(dx) >= deadzone_trans:
            twist_stamped.twist.linear.x = (dx / dt) * linear_scale
        if abs(dy) >= deadzone_trans:
            twist_stamped.twist.linear.y = (dy / dt) * linear_scale
        if abs(dz) >= deadzone_trans:
            twist_stamped.twist.linear.z = (dz / dt) * linear_scale
        
        # 计算角速度（从相对位姿的四元数）
        angular_velocity = self.calculate_angular_velocity(rel_pose.transform.rotation, dt)
        
        # 应用角速度死区和缩放
        angular_magnitude = math.sqrt(
            angular_velocity.x**2 + angular_velocity.y**2 + angular_velocity.z**2)
        if angular_magnitude >= deadzone_rot:
            twist_stamped.twist.angular.x = angular_velocity.x * angular_scale
            twist_stamped.twist.angular.y = angular_velocity.y * angular_scale
            twist_stamped.twist.angular.z = angular_velocity.z * angular_scale
        
        # 限制最大速度
        twist_stamped = self.limit_velocity(twist_stamped)
        
        # 更新上一时刻的状态
        self.last_abs_pose = abs_pose
        self.last_time = current_time
        
        return twist_stamped

    def calculate_angular_velocity(self, rel_rotation, dt):
        """
        从相对旋转四元数计算角速度
        """
        angular_velocity = Vector3()
        
        if dt > 0:
            # 简化计算：基于四元数的虚部估计角速度
            # 实际应该使用更精确的角速度计算
            angular_velocity.x = rel_rotation.x * 2.0 / dt
            angular_velocity.y = rel_rotation.y * 2.0 / dt
            angular_velocity.z = rel_rotation.z * 2.0 / dt
        
        return angular_velocity

    def transform_pose_to_planning_frame(self, vr_pose, target_frame):
        """
        坐标变换到规划坐标系
        """
        try:
            if not self.tf_buffer.can_transform(target_frame, vr_pose.header.frame_id, rclpy.time.Time()):
                self.get_logger().warn(f"等待坐标变换: {vr_pose.header.frame_id} -> {target_frame}")
                return None
            
            transform = self.tf_buffer.lookup_transform(
                target_frame, vr_pose.header.frame_id, rclpy.time.Time())
            
            transformed_pose = tf2_geometry_msgs.do_transform_transform_stamped(vr_pose, transform)
            return transformed_pose
            
        except tf2_ros.TransformException as e:
            self.get_logger().error(f"坐标变换失败: {e}")
            return None

    def limit_velocity(self, twist_stamped):
        """
        限制最大速度
        """
        max_linear = self.get_parameter('max_linear_velocity').value
        max_angular = self.get_parameter('max_angular_velocity').value
        
        twist = twist_stamped.twist
        
        # 限制线速度
        linear_velocity = math.sqrt(twist.linear.x**2 + twist.linear.y**2 + twist.linear.z**2)
        if linear_velocity > max_linear:
            scale = max_linear / linear_velocity
            twist.linear.x *= scale
            twist.linear.y *= scale
            twist.linear.z *= scale
        
        # 限制角速度
        angular_velocity = math.sqrt(twist.angular.x**2 + twist.angular.y**2 + twist.angular.z**2)
        if angular_velocity > max_angular:
            scale = max_angular / angular_velocity
            twist.angular.x *= scale
            twist.angular.y *= scale
            twist.angular.z *= scale
        
        return twist_stamped

    def publish_zero_velocity(self):
        """
        发布零速度命令
        """
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = self.get_parameter('planning_frame').value
        self.twist_pub.publish(twist_stamped)
        self.get_logger().debug("发布零速度命令")

def main():
    rclpy.init()
    node = FR3VRTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.publish_zero_velocity()
        node.get_logger().info("节点关闭 - 发送安全停止命令")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()