import rclpy
from rclpy.node import Node
from herdeye_volcllm_srv.srv import LLMRequest
from herdeye_sensor_msg.msg import MultiSensorData
import json

class LLMClient(Node):
    def __init__(self):
        super().__init__('volc_llm_client')
        # 创建服务客户端
        self.cli = self.create_client(LLMRequest, 'volc_llm')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待LLM服务上线...')
        
        # 创建传感器数据发布者
        self.publisher = self.create_publisher(
            MultiSensorData, 
            'sensors/control_data', 
            10
        )
        self.get_logger().info("传感器数据发布器已创建，主题: /sensor_data")
        
        # 订阅/home_data主题获取water_level和weight
        self.subscription = self.create_subscription(
            MultiSensorData,
            '/sensors/home_data',
            self.sensor_callback,
            10
        )
        self.latest_sensor_data = None
        self.last_water_level = None
        self.last_weight = None
        self.get_logger().info("传感器数据订阅器已创建，主题: /sensors/home_data")
        
        self.get_logger().info("系统已启动，等待传感器数据变化...")
        
        # 创建定时器代替交互循环
        self.timer = self.create_timer(0.1, self.timer_callback)
        
    def sensor_callback(self, msg):
        """处理传感器数据变化"""
        # 首次接收数据
        if self.latest_sensor_data is None:
            self.latest_sensor_data = msg
            self.last_water_level = msg.water_level
            self.last_weight = msg.weight
            self.get_logger().info(f"首次收到传感器数据 - 水位: {msg.water_level}, 重量: {msg.weight}")
            return
        
        # 检查数据是否变化
        if msg.water_level != self.last_water_level or msg.weight != self.last_weight:
            self.latest_sensor_data = msg
            self.last_water_level = msg.water_level
            self.last_weight = msg.weight
            self.get_logger().info(f"传感器数据变化 - 水位: {msg.water_level}, 重量: {msg.weight}")
            
            # 数据变化时立即处理
            self.process_sensor_change()
    
    def timer_callback(self):
        """定时器回调，用于保持节点活跃"""
        # 此方法保持节点运行，但不做任何操作
        # 所有操作由sensor_callback触发
        pass

    def process_sensor_change(self):
        """处理传感器数据变化"""
        if not self.latest_sensor_data:
            self.get_logger().warning("无法处理变化: 没有传感器数据")
            return
        
        # 创建包含用户指令和传感器数据的JSON对象
        request_data = {
            "water_level": self.latest_sensor_data.water_level,
            "weight": self.latest_sensor_data.weight
        }
        
        # 将JSON对象转换为字符串
        request_json = json.dumps(request_data, ensure_ascii=False)
        self.get_logger().info(f"发送给LLM服务的JSON数据: {request_json}")
        
        req = LLMRequest.Request()
        req.request = request_json
        
        future = self.cli.call_async(req)
        future.add_done_callback(self.service_response_callback)
    
    def service_response_callback(self, future):
        """处理服务响应"""
        try:
            response = future.result()
            if response is not None:
                try:
                    # 解析JSON响应
                    response_data = json.loads(response.response)
                    
                    # 打印设备状态
                    self.get_logger().info("\n=== 设备控制指令 ===")
                    light_value = self.print_device_status("灯光", response_data, "LightValue")
                    water_value = self.print_device_status("水泵", response_data, "WaterPumpValue")
                    motor_value = self.print_device_status("电机", response_data, "MotorValue")
                    door_value = self.print_device_status("门", response_data, "DoorValue")
                    
                    # 创建并发布传感器数据消息
                    self.publish_control_data(light_value, water_value, motor_value, door_value)
                    
                except json.JSONDecodeError as e:
                    self.get_logger().error(f"JSON解析失败: {str(e)}")
                    self.get_logger().error(f"原始响应: {response.response}")
            else:
                self.get_logger().error("服务调用失败: 无响应")
        except Exception as e:
            self.get_logger().error(f"服务处理异常: {str(e)}")
    
    def print_device_status(self, device_name, data, key):
        """打印设备状态信息并返回原始值"""
        value = data.get(key, None)
        if value is not None:
            status = "开启" if value else "关闭"
            self.get_logger().info(f"{device_name}状态: {status} (原始值: {value})")
            return value
        else:
            self.get_logger().warning(f"警告: {device_name}状态({key})未在响应中找到")
            return None
    
    def publish_control_data(self, light_value, water_value, motor_value, door_value):
        """发布控制数据到ROS主题"""
        # 创建MultiSensorData消息
        msg = MultiSensorData()
        
        # 设置消息头的时间戳
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "home_control_data"
        
        # 填充设备状态数据
        if light_value is not None:
            msg.light_value = int(light_value)
        if water_value is not None:
            msg.water_pump_value = int(water_value)
        if motor_value is not None:
            msg.motor_value = int(motor_value)
        if door_value is not None:
            msg.door_value = int(door_value)
        
        # 发布消息
        self.publisher.publish(msg)
        self.get_logger().info(f"已发布控制指令: 灯光={msg.light_value}, 水泵={msg.water_pump_value}, "
                              f"电机={msg.motor_value}, 门={msg.door_value}")

def main(args=None):
    rclpy.init(args=args)
    client = LLMClient()
    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()