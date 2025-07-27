#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

class DepthProcessor(Node):
    def __init__(self):
        super().__init__('depth_processor')
        
        # 订阅深度图像话题
        self.subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',  # 替换为你的深度话题
            self.depth_callback,
            10)
        
        self.bridge = CvBridge()
        self.get_logger().info("深度图像处理器已启动...")

    def depth_callback(self, msg):
        try:
            # 将ROS图像消息转换为OpenCV格式
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # 获取图像尺寸
            height, width = depth_image.shape
            self.get_logger().debug(f"收到深度图像: {width}x{height}")
            
            # 处理深度图像
            self.process_depth(depth_image)
            
        except Exception as e:
            self.get_logger().error(f"处理错误: {str(e)}")

    def process_depth(self, depth_image):
        """处理深度图像的核心逻辑"""
        # 1. 转换为米单位（如果原始数据是毫米）
        depth_image_m = depth_image.astype(np.float32) / 1000.0
        
        # 2. 应用高斯滤波去噪
        filtered_depth = cv2.GaussianBlur(depth_image_m, (5, 5), 0)
        
        # 3. 检测最近的物体（用于避障）
        min_depth = np.min(filtered_depth[filtered_depth > 0])  # 忽略零值
        
        # 4. 检测地面平面（用于地形分析）
        ground_mask = self.detect_ground(filtered_depth)
        
        # 5. 检测动物（基于高度和形状）
        animal_detections = self.detect_animals(filtered_depth, ground_mask)
        
        # 6. 可视化结果（调试用）
        self.visualize_results(filtered_depth, ground_mask, animal_detections, min_depth)
        
        # 发布处理结果（实际应用中）
        # self.publish_results(...)

    def detect_ground(self, depth_image):
        """检测地面平面"""
        # 使用RANSAC平面拟合
        # 简化示例：假设地面是深度值在一定范围内的区域
        ground_mask = np.zeros_like(depth_image, dtype=np.uint8)
        ground_mask[(depth_image > 0.5) & (depth_image < 2.0)] = 1  # 0.5-2米范围视为地面
        return ground_mask

    def detect_animals(self, depth_image, ground_mask):
        """检测动物"""
        # 1. 计算高度图
        height_map = depth_image.copy()
        
        # 2. 找出高于地面的物体
        above_ground = np.zeros_like(depth_image)
        above_ground[(ground_mask == 1) & (depth_image > 0)] = depth_image[(ground_mask == 1) & (depth_image > 0)]
        
        # 3. 找出高于地面的连通区域
        _, labels = cv2.connectedComponents((above_ground > 0).astype(np.uint8))
        
        # 4. 根据大小和高度过滤
        detections = []
        for label in np.unique(labels):
            if label == 0:  # 背景
                continue
                
            mask = labels == label
            area = np.sum(mask)
            max_height = np.max(depth_image[mask])
            
            # 过滤条件：面积和高度
            if 500 < area < 5000 and 0.5 < max_height < 2.0:
                detections.append({
                    'mask': mask,
                    'centroid': self.calculate_centroid(mask),
                    'size': area
                })
        
        return detections

    def calculate_centroid(self, mask):
        """计算物体中心点"""
        y, x = np.where(mask)
        return np.mean(x), np.mean(y)

    def visualize_results(self, depth_image, ground_mask, detections, min_depth):
        """可视化处理结果（调试用）"""
        # 归一化深度图像用于显示
        depth_vis = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
        
        # 标记地面
        depth_vis[ground_mask == 1] = (0, 255, 0)  # 绿色表示地面
        
        # 标记检测到的动物
        for detection in detections:
            x, y = detection['centroid']
            cv2.circle(depth_vis, (int(x), int(y)), 10, (0, 0, 255), -1)  # 红色点表示动物中心
            cv2.putText(depth_vis, f"Size: {detection['size']}", (int(x)-50, int(y)-20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # 显示最近物体距离
        cv2.putText(depth_vis, f"Min Depth: {min_depth:.2f}m", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # 显示结果
        cv2.imshow('Depth Processing', depth_vis)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = DepthProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()