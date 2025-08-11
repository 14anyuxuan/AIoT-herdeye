# -*- encoding: utf-8 -*-
import time
import logging
import json
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from IoT_device.client.IoT_client_config import IoTClientConfig
from IoT_device.client.IoT_client import IotClient
from IoT_device.request.services_properties import ServicesProperties

# 导入自定义消息类型
from herdeye_sensor_msg.msg import MultiSensorData

class HuaweiCloudNode(Node):
    def __init__(self):
        super().__init__('huawei_cloud_node')
        
        # 创建回调组
        cloud_cb_group = MutuallyExclusiveCallbackGroup()
        sensor_cb_group = MutuallyExclusiveCallbackGroup()
        
        # 初始化华为云IoT客户端
        self.get_logger().info("Initializing Huawei Cloud IoT Client...")
        client_cfg = IoTClientConfig(
            server_ip='00f6fa7425.st1.iotda-device.cn-north-4.myhuaweicloud.com',
            device_id='685779b332771f177b4495c9_Herdeye_car',
            secret='3b643397fbde3f6c0dd69076fcc19c79',
            is_ssl=False
        )
        
        self.iot_client = IotClient(client_cfg)
        self.iot_client.connect()
        
        # 设置回调函数
        self.iot_client.set_property_set_callback(self.property_set_callback)
        self.iot_client.set_property_get_callback(self.property_get_callback)
        
        # 启动IoT客户端线程
        self.iot_client.start()
        
        # 创建定时器上报属性
        self.timer = self.create_timer(5.0, self.timer_callback, callback_group=cloud_cb_group)
        
        # 订阅温湿度传感器数据
        self.sht30_sub = self.create_subscription(
            MultiSensorData,
            '/sensors/sht30_data',
            self.sht30_callback,
            10,
            callback_group=sensor_cb_group
        )
        
        # 初始化温湿度数据
        self.current_temperature = 0.0
        self.current_humidity = 0.0
        self.last_update_time = self.get_clock().now().seconds_nanoseconds()[0]
        
        self.get_logger().info("Huawei Cloud Node started and subscribed to /sensors/sht30_data")

    def sht30_callback(self, msg):
        """处理温湿度传感器数据回调"""
        # 更新最新数据
        self.current_temperature = msg.sht30_temperature
        self.current_humidity = msg.sht30_humidity
        self.last_update_time = self.get_clock().now().seconds_nanoseconds()[0]
        
        self.get_logger().debug(f"Received SHT30 data: Temp={self.current_temperature:.1f}°C, Hum={self.current_humidity:.1f}%")

    def property_set_callback(self, request_id, payload):
        """处理平台设置属性的请求"""
        self.get_logger().info(f"Received property set request: {payload}")
        
        try:
            data = json.loads(payload)
            for service in data.get('services', []):
                service_id = service.get('service_id')
                self.get_logger().info(f"Service ID: {service_id}")
                
                # 只处理与环境相关的设置
                if service_id == "Environment":
                    for prop, value in service.get('properties', {}).items():
                        self.get_logger().info(f"Property: {prop} = {value}")
                        # 这里可以添加对温湿度设置的处理逻辑
                        # 例如：设置目标温度或湿度
            # 响应平台
            self.iot_client.respond_property_set(request_id, result_code='success')
            
        except Exception as e:
            self.get_logger().error(f"Error processing property set: {str(e)}")
            self.iot_client.respond_property_set(request_id, result_code='failure')

    def property_get_callback(self, request_id, payload):
        """处理平台查询属性的请求"""
        self.get_logger().info(f"Received property get request: {payload}")
        
        try:
            data = json.loads(payload)
            service_id = data.get('service_id')
            self.get_logger().info(f"Querying properties for service: {service_id}")
            
            # 创建属性对象
            service_property = ServicesProperties()
            
            # 只响应环境属性查询
            if service_id == "Environment":
                # 返回温湿度数据
                service_property.add_service_property(service_id, 'temperature', self.current_temperature)
                service_property.add_service_property(service_id, 'humidity', self.current_humidity)
                self.iot_client.respond_property_get(request_id, service_property.service_property)
            else:
                # 对于其他服务ID，返回空响应
                self.iot_client.respond_property_get(request_id, {})
                self.get_logger().warn(f"Unsupported service ID: {service_id}")
            
        except Exception as e:
            self.get_logger().error(f"Error processing property get: {str(e)}")
            # 返回空响应表示失败
            self.iot_client.respond_property_get(request_id, {})

    def report_properties(self):
        """上报温湿度属性到云端"""
        try:
            service_property = ServicesProperties()
            
            # 只添加温湿度数据
            service_property.add_service_property("sht", 'T', self.current_temperature)
            service_property.add_service_property("sht", 'H', self.current_humidity)
            
            # 上报属性
            self.iot_client.report_properties(
                service_properties=service_property.service_property,
                qos=1
            )
            
            self.get_logger().info(f"Reported to cloud: Temp={self.current_temperature:.1f}°C, Hum={self.current_humidity:.1f}%")
            
        except Exception as e:
            self.get_logger().error(f"Error reporting properties: {str(e)}")

    def timer_callback(self):
        """定时器回调函数，用于定期上报温湿度"""
        # 检查数据是否过期
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        if current_time - self.last_update_time > 60:
            self.get_logger().warn("SHT30 data is outdated (over 60 seconds)")
        
        # 上报温湿度
        self.report_properties()

    def destroy_node(self):
        """节点销毁时的清理工作"""
        self.get_logger().info("Shutting down Huawei Cloud Node...")
        if hasattr(self, 'iot_client'):
            # 发送最后的温湿度数据
            try:
                self.report_properties()
            except Exception as e:
                self.get_logger().error(f"Error reporting final properties: {str(e)}")
            
            # 停止客户端
            self.iot_client.stop()
            self.get_logger().info("IoT client stopped")
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = HuaweiCloudNode()
        # 使用多线程执行器
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            node.destroy_node()
    
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()