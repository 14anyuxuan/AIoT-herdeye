#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
from i2cdev import I2C
from sensor_msgs.msg import Temperature, RelativeHumidity
from herdeye_sensor_msg.msg import MultiSensorData
from std_msgs.msg import Header

class SHT30:
    """SHT30温湿度传感器驱动类"""
    DEFAULT_ADDRESS = 0x44  # 默认I2C地址
    MEASURE_HIGH_REP = [0x2C, 0x06]  # 高重复性测量命令
    
    def __init__(self, bus=5, address=DEFAULT_ADDRESS, logger=None):
        """初始化传感器连接"""
        self.bus = bus
        self.address = address
        self.i2c = I2C(address, bus)
        self.logger = logger  # 存储传入的日志记录器
        self.initialize_sensor()
    
    def initialize_sensor(self):
        """初始化传感器"""
        try:
            # 发送软复位命令
            self.i2c.write(bytes([0x30, 0xA2]))
            time.sleep(0.1)
            
            # 清除状态寄存器
            self.i2c.write(bytes([0x30, 0x41]))
            time.sleep(0.05)
            
            if self.logger:
                self.logger.info("传感器初始化完成")
            else:
                print("传感器初始化完成")
        except Exception as e:
            msg = f"初始化警告: {e} - 可能不影响正常操作"
            if self.logger:
                self.logger.warn(msg)
            else:
                print(msg)
    
    def read_temperature_humidity(self):
        """读取温度和湿度数据"""
        try:
            # 发送测量命令
            self.i2c.write(bytes(self.MEASURE_HIGH_REP))
            
            # 等待测量完成
            time.sleep(0.02)
            
            # 读取6字节数据
            data = self.i2c.read(6)
            
            if len(data) != 6:
                raise IOError("读取数据长度错误")
            
            # 解析温度数据
            raw_temp = (data[0] << 8) | data[1]
            # 解析湿度数据
            raw_humidity = (data[3] << 8) | data[4]
            
            # 转换为实际值
            temperature = -45 + 175 * (raw_temp / 65535.0)  # 摄氏度
            humidity = 100 * (raw_humidity / 65535.0)       # 百分比
            
            # 检查数据有效性
            if not (0 <= temperature <= 100) or not (0 <= humidity <= 100):
                raise ValueError("数据超出合理范围")
            
            return temperature, humidity
            
        except Exception as e:
            msg = f"读取错误: {e}"
            if self.logger:
                self.logger.error(msg)
            else:
                print(msg)
            return None, None
    
    def close(self):
        """关闭I2C连接"""
        self.i2c.close()

class SHT30SensorNode(Node):
    """SHT30传感器ROS 2节点"""
    def __init__(self):
        super().__init__('sht30_sensor_node')
        
        # 声明参数
        self.declare_parameter('i2c_bus', 5)
        self.declare_parameter('i2c_address', 0x44)
        self.declare_parameter('update_rate', 1.0)  # Hz
        
        # 获取参数
        i2c_bus = self.get_parameter('i2c_bus').get_parameter_value().integer_value
        i2c_address = self.get_parameter('i2c_address').get_parameter_value().integer_value
        update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        
        # 初始化传感器，并传入节点的日志记录器
        self.sensor = SHT30(
            bus=i2c_bus, 
            address=i2c_address,
            logger=self.get_logger()
        )
        
        
        self.sht30_publisher = self.create_publisher(
            MultiSensorData, 
            'sensors/sht30_data',  # 自定义主题
            10
        )
        
        # 创建定时器
        self.timer = self.create_timer(1.0 / update_rate, self.timer_callback)
        
        self.get_logger().info(f"SHT30传感器节点已启动，更新频率: {update_rate}Hz")
    
    def timer_callback(self):
        """定时器回调函数，读取并发布传感器数据"""
        temperature, humidity = self.sensor.read_temperature_humidity()
        
        if temperature is not None and humidity is not None:
            
            
            # 创建自定义消息
            sht_msg = MultiSensorData()
            
            # 填充消息头
            sht_msg.header = Header()
            sht_msg.header.stamp = self.get_clock().now().to_msg()
            sht_msg.header.frame_id = "sht30_sensor"
            
            # 填充温度数据
            sht_msg.sht30_temperature = temperature + 273.15
            
            # 填充湿度数据
            sht_msg.sht30_humidity = humidity / 100.0
            
            # 发布消息
            self.sht30_publisher.publish(sht_msg)
            
            # 日志记录
            self.get_logger().debug(
                f"发布环境数据: {temperature:.1f}°C, {humidity:.1f}% RH, "
            )
            
    
    def destroy_node(self):
        """节点销毁时的清理工作"""
        self.sensor.close()
        self.get_logger().info("传感器连接已关闭")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = SHT30SensorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()