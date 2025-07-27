#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
import math

class MotionSimulator(Node):
    def __init__(self):
        super().__init__('motion_simulator')
        
        # 初始化参数
        self.x = 0.0  # 初始X位置 (m)
        self.y = 0.0  # 初始Y位置 (m)
        self.th = 0.0  # 初始朝向 (rad)
        self.last_time = self.get_clock().now()
        
        # 创建速度指令订阅器
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.vel_callback,
            10)
        
        # 创建里程计发布器
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # 创建TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 创建定时器以更新位置
        self.timer = self.create_timer(0.05, self.update_position)  # 20Hz更新频率

    def vel_callback(self, msg):
        # 保存当前速度指令
        self.current_vel = msg

    def update_position(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt <= 0:
            return
        
        # 获取当前速度指令
        try:
            vx = self.current_vel.linear.x
            vy = self.current_vel.linear.y
            vth = self.current_vel.angular.z
        except AttributeError:
            # 如果尚未收到速度指令，使用零速度
            vx, vy, vth = 0.0, 0.0, 0.0
        
        # 更新位置和朝向 (欧拉积分)
        delta_x = (vx * math.cos(self.th) - vy * math.sin(self.th)) * dt
        delta_y = (vx * math.sin(self.th) + vy * math.cos(self.th)) * dt
        delta_th = vth * dt
        
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        
        # 标准化角度到 [-pi, pi]
        self.th = math.atan2(math.sin(self.th), math.cos(self.th))
        
        # 发布TF变换 (odom → base_link)
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.th / 2)
        t.transform.rotation.w = math.cos(self.th / 2)
        self.tf_broadcaster.sendTransform(t)
        
        # 发布里程计消息
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = t.transform.rotation
        
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth
        
        self.odom_pub.publish(odom)
        
        # 更新最后时间戳
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = MotionSimulator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()