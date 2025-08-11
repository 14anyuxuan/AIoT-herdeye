#!/usr/bin/env python3
# coding: utf-8

import rclpy
from rclpy.node import Node
import serial
import re
import math
from std_msgs.msg import Header
from herdeye_sensor_msg.msg import MultiSensorData  # 导入自定义消息

class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')
        
        # 声明参数
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('update_rate', 1.0)  # Hz
        
        # 获取参数
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        
        # 初始化变量
        self.utctime = ''
        self.lat = 0.0
        self.ulat = ''
        self.lon = 0.0
        self.ulon = ''
        self.numSv = 0
        self.msl = 0.0
        self.cogt = 0.0
        self.cogm = 0.0
        self.sog = 0.0
        self.kph = 0.0
        self.last_valid_data = None
        
        # 打开串口
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=1.0  # 添加超时设置
            )
            self.get_logger().info(f"GPS Serial Opened! Port={self.serial_port}, Baudrate={self.baud_rate}")
        except Exception as e:
            self.get_logger().error(f"Serial port error: {str(e)}")
            raise
        
        # 创建发布者
        self.multi_sensor_pub = self.create_publisher(MultiSensorData, 'sensors/gps_data', 10)
        
        # 创建定时器
        self.timer = self.create_timer(1.0 / self.update_rate, self.timer_callback)
        
        self.get_logger().info(f"GPS Node Started. Update rate: {self.update_rate}Hz")
    
    def convert_to_degrees(self, value, direction, is_longitude=False):
        """将原始数据转换为度单位，区分经度和纬度"""
        if not value or not direction:
            return 0.0
            
        try:
            # 经度使用3位度数，纬度使用2位度数
            deg_digits = 3 if is_longitude else 2
            
            if len(value) < deg_digits:
                self.get_logger().warn(f"Invalid coordinate value: {value}")
                return 0.0
                
            # 提取度数和分钟
            degrees = float(value[:deg_digits])
            minutes = float(value[deg_digits:])
            
            # 转换为十进制度
            decimal_degrees = degrees + (minutes / 60.0)
            
            # 根据方向调整符号
            if direction in ['S', 'W']:
                decimal_degrees *= -1
                
            return decimal_degrees
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f"Error converting coordinates: {str(e)}, value={value}, direction={direction}")
            return 0.0

    def parse_nmea(self, line):
        """解析NMEA语句"""
        try:
            # 移除换行符和回车符
            line = line.strip()
            
            # 检查校验和（如果存在）
            if '*' in line:
                content, checksum = line.split('*', 1)
                calculated_checksum = 0
                for char in content[1:]:  # 跳过起始'$'
                    calculated_checksum ^= ord(char)
                
                if int(checksum, 16) != calculated_checksum:
                    self.get_logger().debug(f"Checksum mismatch for: {line}")
                    return False
                
                line = content  # 使用不带校验和的部分
            
            # 分割字段
            fields = line.split(',')
            sentence_type = fields[0][3:] if fields[0].startswith('$GP') else fields[0][3:]
            
            # 处理GGA语句 (Global Positioning System Fix Data)
            if sentence_type == 'GGA':
                if len(fields) < 10:
                    self.get_logger().debug("GGA sentence too short")
                    return False
                
                # UTC时间
                self.utctime = fields[1] if len(fields[1]) > 0 else self.utctime
                
                # 纬度（不是经度）
                if len(fields[2]) > 0 and len(fields[3]) > 0:
                    self.lat = self.convert_to_degrees(fields[2], fields[3], is_longitude=False)
                
                # 经度（指定是经度）
                if len(fields[4]) > 0 and len(fields[5]) > 0:
                    self.lon = self.convert_to_degrees(fields[4], fields[5], is_longitude=True)
                
                # 定位质量
                fix_quality = int(fields[6]) if len(fields[6]) > 0 else 0
                
                # 卫星数量
                self.numSv = int(fields[7]) if len(fields[7]) > 0 else self.numSv
                
                # 海拔高度
                if len(fields[9]) > 0:
                    try:
                        self.msl = float(fields[9])
                    except ValueError:
                        pass
                
                return True
            
            # 处理RMC语句 (Recommended Minimum Navigation Information)
            elif sentence_type == 'RMC':
                if len(fields) < 10:
                    self.get_logger().debug("RMC sentence too short")
                    return False
                    
                # 只处理有效数据 ('A'表示有效定位)
                if fields[2] != 'A':  
                    return False
                
                # UTC时间
                self.utctime = fields[1] if len(fields[1]) > 0 else self.utctime
                
                # 纬度（不是经度）
                if len(fields[3]) > 0 and len(fields[4]) > 0:
                    self.lat = self.convert_to_degrees(fields[3], fields[4], is_longitude=False)
                
                # 经度（指定是经度）
                if len(fields[5]) > 0 and len(fields[6]) > 0:
                    self.lon = self.convert_to_degrees(fields[5], fields[6], is_longitude=True)
                
                # 地面速度 (节)
                if len(fields[7]) > 0:
                    try:
                        self.sog = float(fields[7])
                        self.kph = self.sog * 1.852  # 转换为km/h
                    except ValueError:
                        pass
                
                # 地面航向
                if len(fields[8]) > 0:
                    try:
                        self.cogt = float(fields[8])
                    except ValueError:
                        pass
                
                return True
            
            # 处理VTG语句 (Track Made Good and Ground Speed)
            elif sentence_type == 'VTG':
                if len(fields) < 9:
                    self.get_logger().debug("VTG sentence too short")
                    return False
                
                # 地面航向
                if len(fields[1]) > 0:
                    try:
                        self.cogt = float(fields[1])
                    except ValueError:
                        pass
                
                # 地面速度 (km/h)
                if len(fields[7]) > 0:
                    try:
                        self.kph = float(fields[7])
                    except ValueError:
                        pass
                
                return True
            
            # 可以添加其他NMEA语句类型的处理
            else:
                # self.get_logger().debug(f"Ignored NMEA sentence: {sentence_type}")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Error parsing NMEA: {str(e)}, line: {line}")
            return False

    def timer_callback(self):
        """定时器回调函数，读取并发布GPS数据"""
        try:
            # 读取所有可用数据行
            data_available = False
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('ascii', errors='ignore')
                if self.parse_nmea(line):
                    data_available = True
            
            # 如果没有解析到有效数据，跳过本次发布
            if not data_available:
                return
            
            # 创建时间戳
            timestamp = self.get_clock().now().to_msg()
            
            # 创建并填充MultiSensorData消息
            msg = MultiSensorData()
            msg.header = Header()
            msg.header.stamp = timestamp
            msg.header.frame_id = 'gps'
            
            # 填充GPS数据字段
            msg.gps_lat = float(self.lat)
            msg.gps_lng = float(self.lon)
            msg.gps_altitude = float(self.msl)
            msg.gps_cogt = float(self.cogt)
            msg.gps_speed = float(self.kph)
            msg.gps_satellites = int(self.numSv)
            
            # 发布消息
            self.multi_sensor_pub.publish(msg)
            self.last_valid_data = msg
            
            # 记录日志
            self.get_logger().info(
                f"Published GPS: Lat={self.lat:.6f}, Lon={self.lon:.6f}, "
                f"Alt={self.msl:.1f}m, Speed={self.kph:.1f}km/h, "
                f"Heading={self.cogt:.1f}°, Sats={self.numSv}"
            )
            
        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {str(e)}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {str(e)}")

    def __del__(self):
        """析构函数，关闭串口"""
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("GPS serial closed.")

def main(args=None):
    rclpy.init(args=args)
    try:
        gps_node = GPSNode()
        rclpy.spin(gps_node)
    except Exception as e:
        if 'gps_node' in locals():
            gps_node.get_logger().error(f"Node error: {str(e)}")
        else:
            print(f"Node initialization failed: {str(e)}")
    finally:
        if 'gps_node' in locals():
            gps_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()