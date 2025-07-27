#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import cv2
import numpy as np

class DetectionSubscriber(Node):
    def __init__(self):
        super().__init__('detection_subscriber')
        
        # 创建图像订阅
        self.image_sub = self.create_subscription(
            Image,
            '/detected_image',  # 与发布节点的话题名称一致
            self.image_callback,
            10
        )
        
        # 创建检测结果订阅
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/detections',  # 与发布节点的话题名称一致
            self.detection_callback,
            10
        )
        
        # 初始化CV桥
        self.bridge = CvBridge()
        self.current_image = None
        self.current_detections = []
        
        # 创建显示窗口
        cv2.namedWindow('Detection Preview', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Detection Preview', 800, 600)
        
        self.get_logger().info("检测订阅节点已启动")

    def image_callback(self, msg):
        try:
            # 转换ROS图像消息为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.current_image = cv_image.copy()
            self.visualize_detections()
        except Exception as e:
            self.get_logger().error(f"图像转换失败: {str(e)}")

    def detection_callback(self, msg):
        # 解析检测结果
        detections = []
        for detection in msg.detections:
            bbox = detection.bbox
            for result in detection.results:
                detections.append({
                    'class': result.hypothesis.class_id,
                    'score': result.hypothesis.score,
                    'center_x': bbox.center.position.x,
                    'center_y': bbox.center.position.y,
                    'width': bbox.size_x,
                    'height': bbox.size_y
                })
        self.current_detections = detections
        self.print_detections()

    def print_detections(self):
        # 在终端打印检测结果
        if len(self.current_detections) > 0:
            print("\n=== 当前帧检测结果 ===")
            for idx, det in enumerate(self.current_detections, 1):
                print(f"目标 {idx}:")
                print(f"  类别: {det['class']}")
                print(f"  置信度: {det['score']:.2f}")
                print(f"  位置: ({det['center_x']:.1f}, {det['center_y']:.1f})")
                print(f"  尺寸: {det['width']:.1f}x{det['height']:.1f}")
            print("=====================\n")

    def visualize_detections(self):
        # 在图像上绘制检测框
        if self.current_image is not None and len(self.current_detections) > 0:
            img = self.current_image.copy()
            for det in self.current_detections:
                x1 = int(det['center_x'] - det['width']/2)
                y1 = int(det['center_y'] - det['height']/2)
                x2 = int(det['center_x'] + det['width']/2)
                y2 = int(det['center_y'] + det['height']/2)
                
                # 绘制矩形框
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                # 添加标签
                label = f"{det['class']} {det['score']:.2f}"
                cv2.putText(img, label, (x1, y1-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
            # 显示图像
            cv2.imshow('Detection Preview', img)
            cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DetectionSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()