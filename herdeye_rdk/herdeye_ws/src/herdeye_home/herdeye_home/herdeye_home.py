#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import json
import time
from herdeye_sensor_msg.msg import MultiSensorData
from std_msgs.msg import Header

class HomeSensorNode(Node):
    def __init__(self):
        super().__init__('home_sensor_node')
        
        # 声明ROS参数
        self.declare_parameter('uart_device', '/dev/ttyS1')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('frame_id', 'sensor_frame')
        self.declare_parameter('debug_raw', False)
        self.declare_parameter('max_retries', 3)
        self.declare_parameter('ack_timeout', 0.1)  # 100ms ACK超时
        
        # 获取参数值
        uart_dev = self.get_parameter('uart_device').get_parameter_value().string_value
        baudrate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.debug_raw = self.get_parameter('debug_raw').get_parameter_value().bool_value
        self.max_retries = self.get_parameter('max_retries').get_parameter_value().integer_value
        self.ack_timeout = self.get_parameter('ack_timeout').get_parameter_value().double_value
        
        # 序列号计数器
        self.sequence_number = 0
        
        # 初始化串口连接
        self.ser = None
        try:
            self.ser = serial.Serial(
                uart_dev,
                baudrate,
                timeout=0.1,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            self.get_logger().info(f"Connected to UART: {uart_dev} at {baudrate} baud")
        except Exception as e:
            self.get_logger().error(f"Serial connection failed: {str(e)}")
            raise RuntimeError("Serial connection failed") from e
        
        # 创建发布者
        self.publisher = self.create_publisher(MultiSensorData, 'sensors/home_data', 10)
        self.get_logger().info("Publisher created for topic: home_data")
        
        # 订阅执行器状态数据
        self.control_sub = self.create_subscription(
            MultiSensorData,
            '/sensors/control_data',
            self.control_callback,
            10
        )
        self.get_logger().info("Subscribed to control data")
        
        # 保存帧ID
        self.frame_id = frame_id
        
        # 创建定时器读取串口数据
        self.timer = self.create_timer(0.01, self.read_serial_data)
        
        # 数据缓冲区
        self.buffer = bytearray()
        
        # 用于ACK等待的临时缓冲区
        self.ack_buffer = bytearray()
        
    def reconnect_serial(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except:
                pass
        
        try:
            self.ser = serial.Serial(
                self.get_parameter('uart_device').get_parameter_value().string_value,
                self.get_parameter('baud_rate').get_parameter_value().integer_value,
                timeout=0.1
            )
            self.get_logger().info("Reconnected to serial port")
            return True
        except Exception as e:
            self.get_logger().error(f"Reconnect failed: {str(e)}")
            return False
    
    def control_callback(self, msg):
        try:
            # 从消息中提取传感器数据
            light = msg.light
            waterpump = msg.waterpump 
            motor = msg.motor
            door = msg.door
            heat = msg.heat
            fan = msg.fan
            
            # 添加详细日志
            self.get_logger().info(f"Received actuator status data: light={light}, waterpump={waterpump}, motor={motor}, door={door}, heat={heat}, fan={fan}")
            
            # 封装为JSON字符串
            self.sequence_number = (self.sequence_number + 1) % 1000
            sensor_data = {
                "seq": self.sequence_number,
                "LightValue": light,
                "WaterPumpValue": waterpump,
                "MotorValue": motor,
                "DoorValue": door,
                "HeatValue": heat,
                "FanValue": fan
            }
            json_data = json.dumps(sensor_data)
            
            # 在JSON字符串末尾添加\r\n
            json_with_end = json_data + "\r\n"
            encoded_data = json_with_end.encode('UTF-8')
            
            # 检查串口是否可用
            if not (self.ser and self.ser.is_open):
                self.get_logger().warning("Serial port not available, attempting reconnect")
                if not self.reconnect_serial():
                    self.get_logger().error("Reconnect failed, skipping send")
                    return
            
            # 尝试多次发送（带ACK确认）
            send_success = False
            for attempt in range(1, self.max_retries + 1):
                try:
                    # 清空输入缓冲区
                    self.ser.reset_input_buffer()
                    self.ack_buffer = bytearray()
                    
                    # 通过串口发送
                    self.get_logger().info(f"Sending command (attempt {attempt}/{self.max_retries}): {json_with_end.strip()}")
                    write_num = self.ser.write(encoded_data)
                    self.ser.flush()  # 确保数据发送完成
                    
                    # 等待ACK响应
                    start_time = time.time()
                    ack_received = False
                    expected_ack = f"ACK{self.sequence_number}\r\n".encode('UTF-8')
                    
                    while time.time() - start_time < self.ack_timeout:
                        # 读取所有可用数据
                        if self.ser.in_waiting > 0:
                            data = self.ser.read(self.ser.in_waiting)
                            self.ack_buffer.extend(data)
                            
                            # 检查是否收到完整的ACK
                            if expected_ack in self.ack_buffer:
                                ack_received = True
                                break
                        
                        # 短暂休眠减少CPU占用
                        time.sleep(0.001)
                    
                    if ack_received:
                        self.get_logger().info(f"ACK received for sequence {self.sequence_number}")
                        send_success = True
                        break
                    else:
                        self.get_logger().warning(f"ACK timeout on attempt {attempt}")
                
                except serial.SerialException as e:
                    self.get_logger().error(f"Serial error during attempt {attempt}: {str(e)}")
                    # 尝试重新连接
                    self.reconnect_serial()
            
            if not send_success:
                self.get_logger().error(f"Failed to receive ACK after {self.max_retries} attempts")
            
            # 将ACK缓冲区数据移回主缓冲区
            if self.ack_buffer:
                self.buffer = self.ack_buffer + self.buffer

        except Exception as e:
            self.get_logger().error(f"Error handling actuator status data: {str(e)}", exc_info=True)
    
    def read_serial_data(self):
        # 读取所有可用数据
        try:
            if self.ser and self.ser.is_open and self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting)
                self.buffer.extend(data)
        except serial.SerialException as e:
            self.get_logger().error(f"Serial read error: {str(e)}")
            self.reconnect_serial()
            return
        
        # 处理缓冲区中的数据
        while b'\n' in self.buffer:
            # 找到第一个换行符
            idx = self.buffer.index(b'\n')
            # 提取一行数据
            line = self.buffer[:idx]
            # 移除已处理的数据
            self.buffer = self.buffer[idx+1:]
            
            # 处理这一行数据
            self.process_line(line)
    
    def process_line(self, line_bytes):
        # 跳过空行
        if not line_bytes:
            return
            
        try:
            # 解码数据
            decoded_str = line_bytes.decode('UTF-8').strip()
            
            # 调试输出原始数据
            if self.debug_raw:
                self.get_logger().info(f"Raw data: {decoded_str}")
            
            # 检查是否为ACK响应（不处理）
            if decoded_str.startswith("ACK"):
                return
                
            # 解析JSON对象
            json_data = json.loads(decoded_str)
            
            # 检查是否包含water_level属性
            if "water_level" not in json_data:
                self.get_logger().warn("JSON does not contain 'water_level' property")
                return

            if "weight" not in json_data:
                self.get_logger().warn("JSON does not contain 'weight' property")
                return
                
            # 创建并填充消息
            msg = MultiSensorData()
            
            # 设置消息头
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            
            # 填充数据
            msg.water_level = int(json_data["water_level"])
            msg.weight = int(json_data["weight"])
            
            # 发布消息
            self.publisher.publish(msg)
            self.get_logger().debug(f"Published water level: {msg.water_level}, weight: {msg.weight}")
            
        except UnicodeDecodeError:
            self.get_logger().warn(f"Invalid UTF-8 data: {line_bytes.hex(' ').upper()}")
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"JSON parse error: {str(e)}")
            self.get_logger().warn(f"Problem data: {line_bytes.decode('UTF-8', errors='replace')}")
        except (KeyError, ValueError, TypeError) as e:
            self.get_logger().warn(f"Data conversion error: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = HomeSensorNode()
        rclpy.spin(node)
    except RuntimeError as e:
        if 'node' in locals():
            node.get_logger().error(f"Node initialization failed: {str(e)}")
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            if hasattr(node, 'ser') and node.ser and node.ser.is_open:
                node.ser.close()
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()