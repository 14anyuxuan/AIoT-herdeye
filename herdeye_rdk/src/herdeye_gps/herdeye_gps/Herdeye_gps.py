#!/usr/bin/env python3
# coding: utf-8

import rclpy
from rclpy.node import Node
import serial
import re
import time
import math
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Header
from geometry_msgs.msg import TwistStamped
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
        self.gps_t = 0  # 用于标记GGA是否已解析
        
        # 打开串口
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate)
            if self.ser.isOpen():
                self.get_logger().info(f"GPS Serial Opened! Port={self.serial_port}, Baudrate={self.baud_rate}")
            else:
                self.get_logger().error("GPS Serial Open Failed!")
                raise Exception("GPS Serial Open Failed")
        except Exception as e:
            self.get_logger().error(f"Serial port error: {str(e)}")
            raise
        
        # 创建发布者
        self.fix_pub = self.create_publisher(NavSatFix, 'gps/fix', 10)
        self.vel_pub = self.create_publisher(TwistStamped, 'gps/velocity', 10)
        self.status_pub = self.create_publisher(NavSatStatus, 'gps/status', 10)
        # 新增MultiSensorData发布者
        self.multi_sensor_pub = self.create_publisher(MultiSensorData, 'sensors/gps_data', 10)
        
        # 创建定时器
        self.timer = self.create_timer(1.0 / self.update_rate, self.timer_callback)
        
        self.get_logger().info(f"GPS Node Started. Update rate: {self.update_rate}Hz")
    
    def convert_to_degrees(self, in_data1, in_data2):
        """将原始数据转换为度单位"""
        len_data1 = len(in_data1)
        str_data2 = "%05d" % int(in_data2)
        temp_data = int(in_data1)
        symbol = 1
        if temp_data < 0:
            symbol = -1
        degree = int(temp_data / 100.0)
        str_decimal = str(in_data1[len_data1-2]) + str(in_data1[len_data1-1]) + str(str_data2)
        f_degree = int(str_decimal)/60.0/100000.0
        if symbol > 0:
            result = degree + f_degree
        else:
            result = degree - f_degree
        return result

    def gps_read(self):
        """从串口读取并解析GPS数据"""
        if self.ser.inWaiting():
            if self.ser.read(1) == b'G':
                if self.ser.inWaiting():
                    if self.ser.read(1) == b'N':
                        if self.ser.inWaiting():
                            choice = self.ser.read(1)
                            if choice == b'G':
                                if self.ser.inWaiting():
                                    if self.ser.read(1) == b'G':
                                        if self.ser.inWaiting():
                                            if self.ser.read(1) == b'A':
                                                # 读取GGA数据
                                                GGA = self.ser.read(70)
                                                try:
                                                    GGA_str = GGA.decode('ascii', errors='ignore')
                                                except:
                                                    return 0
                                                GGA_g = re.findall(r"\w+(?=,)|(?<=,)\w+", GGA_str)
                                                if len(GGA_g) < 13:
                                                    self.get_logger().debug("GPS no found (GGA)")
                                                    self.gps_t = 0
                                                    return 0
                                                else:
                                                    self.utctime = GGA_g[0]
                                                    try:
                                                        self.lat = self.convert_to_degrees(str(GGA_g[2]), str(GGA_g[3]))
                                                        self.ulat = GGA_g[4]
                                                        self.lon = self.convert_to_degrees(str(GGA_g[5]), str(GGA_g[6]))
                                                        self.ulon = GGA_g[7]
                                                        self.numSv = int(GGA_g[9])
                                                        self.msl = float(GGA_g[12] + '.' + GGA_g[13])
                                                    except Exception as e:
                                                        self.get_logger().error(f"Error parsing GGA: {e}")
                                                        return 0
                                                    self.gps_t = 1
                                                    return 1
                            elif choice == b'V':
                                if self.ser.inWaiting():
                                    if self.ser.read(1) == b'T':
                                        if self.ser.inWaiting():
                                            if self.ser.read(1) == b'G':
                                                if self.gps_t == 1:
                                                    # 读取VTG数据
                                                    VTG = self.ser.read(40)
                                                    try:
                                                        VTG_str = VTG.decode('ascii', errors='ignore')
                                                    except:
                                                        return 0
                                                    VTG_g = re.findall(r"\w+(?=,)|(?<=,)\w+", VTG_str)
                                                    if len(VTG_g) < 9:
                                                        self.get_logger().debug("GPS no found (VTG)")
                                                        return 0
                                                    try:
                                                        self.cogt = float(VTG_g[0] + '.' + VTG_g[1])
                                                        if VTG_g[3] == 'M':
                                                            self.cogm = 0.0
                                                            self.sog = float(VTG_g[4] + '.' + VTG_g[5])
                                                            self.kph = float(VTG_g[7] + '.' + VTG_g[8])
                                                        else:
                                                            self.cogm = float(VTG_g[3] + '.' + VTG_g[4])
                                                            self.sog = float(VTG_g[6] + '.' + VTG_g[7])
                                                            self.kph = float(VTG_g[9] + '.' + VTG_g[10])
                                                    except Exception as e:
                                                        self.get_logger().error(f"Error parsing VTG: {e}")
                                                    return 2
        return 0

    def timer_callback(self):
        """定时器回调函数，读取并发布GPS数据"""
        # 读取GPS数据
        result = self.gps_read()
        
        if result > 0:
            # 创建时间戳
            timestamp = self.get_clock().now().to_msg()
            
            # 调整经纬度符号
            latitude = self.lat if self.ulat == 'N' else -self.lat
            longitude = self.lon if self.ulon == 'E' else -self.lon
            
            # ========== 发布到MultiSensorData ==========
            multi_msg = MultiSensorData()
            multi_msg.header = Header()
            multi_msg.header.stamp = timestamp
            multi_msg.header.frame_id = 'gps'
            
            # 填充GPS数据
            multi_msg.gps_lat = latitude
            multi_msg.gps_lng = longitude
            multi_msg.gps_altitude = self.msl
            multi_msg.gps_cogt = self.cogt  # 真北航向角（度）
            multi_msg.gps_speed = self.kph     # 速度（km/h）
            multi_msg.gps_satellites = self.numSv
            
            # 发布消息
            self.multi_sensor_pub.publish(multi_msg)
            # ==========================================
            
            # 发布位置信息 (NavSatFix)
            fix_msg = NavSatFix()
            fix_msg.header = Header()
            fix_msg.header.stamp = timestamp
            fix_msg.header.frame_id = 'gps'
            
            # 设置状态信息
            status = NavSatStatus()
            status.status = NavSatStatus.STATUS_FIX if self.numSv >= 3 else NavSatStatus.STATUS_NO_FIX
            status.service = NavSatStatus.SERVICE_GPS
            fix_msg.status = status
            
            # 设置位置数据
            fix_msg.latitude = latitude
            fix_msg.longitude = longitude
            fix_msg.altitude = self.msl
            
            # 设置位置协方差（未知）
            fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            
            self.fix_pub.publish(fix_msg)
            
            # 发布速度信息 (TwistStamped)
            vel_msg = TwistStamped()
            vel_msg.header = Header()
            vel_msg.header.stamp = timestamp
            vel_msg.header.frame_id = 'gps'
            
            # 将航向角转换为弧度（ROS使用弧度）
            heading_rad = self.cogt * math.pi / 180.0
            
            # 计算速度分量（东向和北向）
            # 注意：速度单位为m/s，1节 = 0.514444 m/s
            speed_mps = self.sog * 0.514444
            vel_msg.twist.linear.x = speed_mps * math.sin(heading_rad)  # 东向速度
            vel_msg.twist.linear.y = speed_mps * math.cos(heading_rad)  # 北向速度
            
            self.vel_pub.publish(vel_msg)
            
            # 可选：发布状态信息
            status_msg = NavSatStatus()
            status_msg.header = Header()
            status_msg.header.stamp = timestamp
            status_msg.header.frame_id = 'gps'
            status_msg.status = status.status
            status_msg.service = status.service
            self.status_pub.publish(status_msg)
            
            # 记录日志
            self.get_logger().debug(
                f"GPS Data: Lat={latitude:.6f}, Lon={longitude:.6f}, "
                f"Alt={self.msl:.1f}m, Speed={self.kph:.1f}km/h, "
                f"Heading={self.cogt:.1f}°, Sats={self.numSv}"
            )

    def __del__(self):
        """析构函数，关闭串口"""
        if hasattr(self, 'ser') and self.ser.isOpen():
            self.ser.close()
            self.get_logger().info("GPS serial closed.")

def main(args=None):
    rclpy.init(args=args)
    try:
        gps_node = GPSNode()
        rclpy.spin(gps_node)
    except Exception as e:
        gps_node.get_logger().error(f"Node error: {str(e)}")
    finally:
        if gps_node:
            gps_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()