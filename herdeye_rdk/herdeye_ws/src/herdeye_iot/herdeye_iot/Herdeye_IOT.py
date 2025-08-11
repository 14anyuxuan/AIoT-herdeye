#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import subprocess
import shlex
import signal
import os
import threading
import time
import json
import re
import logging
from std_msgs.msg import String
from herdeye_sensor_msg.msg import MultiSensorData  # 导入传感器消息类型

class IoTBridge(Node):
    def __init__(self):
        super().__init__('iot_bridge')
        
        # 设置详细的日志级别
        self.logger = self.get_logger()
        self.logger.set_level(logging.DEBUG)
        
        # 声明可执行文件路径参数
        self.declare_parameter('executable_path', '/home/root/onesdk_v0.3.0/build/examples/onesdk_iot/onesdk_iot')
        self.executable_path = self.get_parameter('executable_path').value
        
        # 添加阈值参数
        self.declare_parameter('water_level_threshold_low', 10)  # 低水位阈值
        self.declare_parameter('water_level_threshold_high', 30)  # 高水位阈值
        self.declare_parameter('weight_threshold_low', 5)  # 低重量阈值
        self.declare_parameter('weight_threshold_high', 20)  # 高重量阈值
        self.declare_parameter('pump_duration', 5.0)  # 水泵开启持续时间（秒）
        # 添加风扇和加热器阈值参数
        self.declare_parameter('fan_threshold', 28.0)  # 风扇开启温度阈值
        self.declare_parameter('heater_threshold', 18.0)  # 加热器开启温度阈值
        self.declare_parameter('temperature_hysteresis', 2.0)  # 温度控制回差
        
        # 获取阈值参数值
        self.water_level_threshold_low = self.get_parameter('water_level_threshold_low').value
        self.water_level_threshold_high = self.get_parameter('water_level_threshold_high').value
        self.weight_threshold_low = self.get_parameter('weight_threshold_low').value
        self.weight_threshold_high = self.get_parameter('weight_threshold_high').value
        self.pump_duration = self.get_parameter('pump_duration').value
        self.fan_threshold = self.get_parameter('fan_threshold').value
        self.heater_threshold = self.get_parameter('heater_threshold').value
        self.temperature_hysteresis = self.get_parameter('temperature_hysteresis').value
        
        self.logger.info(f"水位阈值: 低={self.water_level_threshold_low}%, 高={self.water_level_threshold_high}%")
        self.logger.info(f"重量阈值: 低={self.weight_threshold_low}g, 高={self.weight_threshold_high}g")
        self.logger.info(f"水泵持续时长: {self.pump_duration}秒")
        self.logger.info(f"风扇开启阈值: {self.fan_threshold}°C")
        self.logger.info(f"加热器开启阈值: {self.heater_threshold}°C")
        self.logger.info(f"温度控制回差: {self.temperature_hysteresis}°C")
        
        # 检查文件是否存在和可执行
        if not os.path.isfile(self.executable_path):
            self.logger.error(f"Executable not found: {self.executable_path}")
            raise FileNotFoundError(f"Executable not found: {self.executable_path}")
        if not os.access(self.executable_path, os.X_OK):
            self.logger.error(f"Missing execute permission: {self.executable_path}")
            raise PermissionError(f"Missing execute permission: {self.executable_path}")
        
        self.logger.info(f"Starting C program: {self.executable_path}")
        
        # 创建消息发布器
        self.property_set_pub = self.create_publisher(String, 'iot/property_set', 10)
        self.custom_topic_pub = self.create_publisher(String, 'iot/custom_topic', 10)
        self.service_call_pub = self.create_publisher(String, 'iot/service_call', 10)
        
        # 创建控制器数据发布者
        self.home_control_publisher = self.create_publisher(
            MultiSensorData, 
            'sensors/control_data', 
            10
        )
        self.get_logger().info("topic: sensors/control_data")
        
        # 订阅SHT30传感器数据
        self.sht30_sub = self.create_subscription(
            MultiSensorData,
            'sensors/sht30_data',
            self.sht30_callback,
            10
        )
        self.logger.info("Subscribed to SHT30 sensor data")
        
        # 订阅home传感器数据
        self.home_sub = self.create_subscription(
            MultiSensorData,
            'sensors/home_data',
            self.home_callback,
            10
        )
        self.logger.info("Subscribed to home sensor data")
        
        # 订阅gps传感器数据
        self.gps_sub = self.create_subscription(
            MultiSensorData,
            'sensors/gps_data',
            self.gps_callback,
            10
        )
        self.logger.info("Subscribed to gps sensor data")
        
        # 订阅控制器状态数据
        self.control_sub = self.create_subscription(
            MultiSensorData,
            'sensors/control_data',
            self.control_callback,
            10
        )
        self.logger.info("Subscribed to control sensor data")
        
        # 初始化控制状态
        self.control_state = MultiSensorData()
        # 初始化所有控制值为0
        self.control_state.motor = 0
        self.control_state.waterpump = 0
        self.control_state.light = 0
        self.control_state.door = 0
        self.control_state.fan = 0  
        self.control_state.heat = 0 
        
        # 水泵定时器相关
        self.pump_timer = None
        self.pump_end_time = 0
        self.pump_auto_mode = False  # 水泵是否处于自动模式
        
        # 连接状态跟踪
        self.cloud_connected = False
        self.model_initialized = False
        self.connection_timeout = 30  # 等待连接的超时时间（秒）
        self.start_time = time.time()
        
        # 警告过滤
        self.warning_filter = [
            re.compile(r'_lws_smd_msg_send: rejecting message on queue depth'),
            re.compile(r'\[W\]: _lws_smd_msg_send: rejecting message on queue depth'),
            re.compile(r'\[W\]: lws_smd_msg_send: rejecting message on queue depth'),
            re.compile(r'queue depth \d+'),
        ]
        
        # 消息解析计数器
        self.message_counter = {
            'success': 0,
            'failures': 0,
            'last_failure': ''
        }
        
        try:
            # 启动C程序（添加标准输入管道）
            self.process = subprocess.Popen(
                [self.executable_path],
                stdin=subprocess.PIPE,  # 添加标准输入管道
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,  # 行缓冲
                preexec_fn=os.setsid
            )
            
            # 启动输出监控线程
            self.stdout_thread = threading.Thread(target=self.handle_stdout)
            self.stdout_thread.daemon = True
            self.stdout_thread.start()
            
            self.stderr_thread = threading.Thread(target=self.handle_stderr)
            self.stderr_thread.daemon = True
            self.stderr_thread.start()
            
            # 启动连接状态检查定时器
            self.connection_check_timer = self.create_timer(1.0, self.check_connection_status)
            
            # 启动水泵状态检查定时器
            self.pump_check_timer = self.create_timer(1.0, self.check_pump_status)
            
            # 警告计数器
            self.warning_count = 0
            self.last_warning_time = 0
            
        except Exception as e:
            self.logger.error(f"Failed to start process: {str(e)}")
            raise

    ########################################创建传感器数据回调函数#################################################

    def sht30_callback(self, msg):
        """SHT30传感器数据回调函数"""
        try:
            # 从消息中提取传感器数据
            temperature_k = msg.sht30_temperature  # 开尔文温度
            humidity_ratio = msg.sht30_humidity    # 湿度比值 (0-1)
            
            # 转换为摄氏度（开尔文转摄氏度）
            temperature_c = temperature_k - 273.15
            
            # 转换为百分比湿度
            humidity_percent = humidity_ratio * 100.0
            
            self.logger.debug(f"收到传感器数据: 温度={temperature_c:.2f}°C, 湿度={humidity_percent:.2f}%")
            
            # 创建要发送给C程序的JSON数据
            sensor_data = {
                "sensor_type": "sht30",
                "temperature": temperature_c,
                "humidity": humidity_percent,
                "timestamp": time.time()
            }
            
            self.send_to_c_program(json.dumps(sensor_data))
            
            # 添加自动温控逻辑
            self.auto_temperature_control(temperature_c)
            
        except Exception as e:
            self.logger.error(f"处理传感器数据失败: {str(e)}")
            
    def auto_temperature_control(self, temperature_c):
        """根据温度自动控制风扇和加热器"""
        try:
            # 风扇控制逻辑
            if temperature_c > self.fan_threshold:
                if self.control_state.fan == 0:
                    self.logger.info(f"温度过高 ({temperature_c:.1f}°C > {self.fan_threshold}°C)，开启风扇")
                    self.control_fan(True)
            elif temperature_c < (self.fan_threshold - self.temperature_hysteresis):
                if self.control_state.fan == 1:
                    self.logger.info(f"温度降低 ({temperature_c:.1f}°C < {self.fan_threshold-self.temperature_hysteresis:.1f}°C)，关闭风扇")
                    self.control_fan(False)
                    
            # 加热器控制逻辑
            if temperature_c < self.heater_threshold:
                if self.control_state.heat == 0:
                    self.logger.info(f"温度过低 ({temperature_c:.1f}°C < {self.heater_threshold}°C)，开启加热器")
                    self.control_heater(True)
            elif temperature_c > (self.heater_threshold + self.temperature_hysteresis):
                if self.control_state.heat == 1:
                    self.logger.info(f"温度升高 ({temperature_c:.1f}°C > {self.heater_threshold+self.temperature_hysteresis:.1f}°C)，关闭加热器")
                    self.control_heater(False)
                    
        except Exception as e:
            self.logger.error(f"自动温控失败: {str(e)}")
            
    def control_fan(self, state):
        """控制风扇状态"""
        try:
            # 如果状态没有变化，则不需要操作
            if self.control_state.fan == int(state):
                return
            
            # 更新控制状态
            self.control_state.fan = 1 if state else 0
            
            # 创建并发布控制消息
            control_msg = MultiSensorData()
            control_msg.light = self.control_state.light
            control_msg.waterpump = self.control_state.waterpump
            control_msg.motor = self.control_state.motor
            control_msg.door = self.control_state.door
            control_msg.fan = self.control_state.fan      # 风扇状态
            control_msg.heat = self.control_state.heat    # 加热器状态
            
            self.home_control_publisher.publish(control_msg)
            self.logger.info(f"已{'开启' if state else '关闭'}风扇")
            
        except Exception as e:
            self.logger.error(f"控制风扇失败: {str(e)}")
            
    def control_heater(self, state):
        """控制加热器状态"""
        try:
            # 如果状态没有变化，则不需要操作
            if self.control_state.heat == int(state):
                return
            
            # 更新控制状态
            self.control_state.heat = 1 if state else 0
            
            # 创建并发布控制消息
            control_msg = MultiSensorData()
            control_msg.light = self.control_state.light
            control_msg.waterpump = self.control_state.waterpump
            control_msg.motor = self.control_state.motor
            control_msg.door = self.control_state.door
            control_msg.fan = self.control_state.fan    
            control_msg.heat = self.control_state.heat  
            
            self.home_control_publisher.publish(control_msg)
            self.logger.info(f"已{'开启' if state else '关闭'}加热器")
            
        except Exception as e:
            self.logger.error(f"控制加热器失败: {str(e)}")
            
    def home_callback(self, msg):
        """home传感器数据回调函数"""
        try:
            # 检查消息是否有water_level字段
            if not hasattr(msg, 'water_level'):
                self.logger.error("home消息中没有water_level字段！")
                self.logger.debug(f"完整消息内容: {msg}")
                return
                
            if not hasattr(msg, 'weight'):
                self.logger.error("home消息中没有weight字段！")
                self.logger.debug(f"完整消息内容: {msg}")
                return
                
            # 从消息中提取传感器数据
            water_level = msg.water_level
            weight = msg.weight
            
            # 添加详细日志
            self.logger.debug(f"收到home传感器数据: water_level={water_level}%, weight={weight}g")
            
            # 创建要发送给C程序的JSON数据
            sensor_data = {
                "sensor_type": "home",
                "water_level": water_level,
                "weight": weight,
            }
            
            self.send_to_c_program(json.dumps(sensor_data))
            
            # 进行阈值判断并自动控制设备
            self.check_thresholds_and_control(water_level, weight)
            
        except Exception as e:
            self.logger.error(f"处理home传感器数据失败: {str(e)}", exc_info=True)
            
            
    def gps_callback(self, msg):
        """weight传感器数据回调函数"""
        try:
                
            # 从消息中提取传感器数据
            gps_lng = msg.gps_lng
            gps_lat = msg.gps_lat
            gps_altitude = msg.gps_altitude
            gps_cogt = msg.gps_cogt
            gps_speed = msg.gps_speed
            gps_satellites = msg.gps_satellites
            
            
            # 添加详细日志
            self.logger.debug(f"收到gps传感器数据: gps_lng={gps_lng}, gps_lat={gps_lat}, gps_altitude={gps_altitude}, gps_cogt={gps_cogt}, gps_speed={gps_speed}, gps_satellites={gps_satellites}")
            
            # 创建要发送给C程序的JSON数据
            sensor_data = {
                "sensor_type": "gps",
                "gps_lng": gps_lng,
                "gps_lat": gps_lat,
                "gps_altitude": gps_altitude,
                "gps_cogt": gps_cogt,
                "gps_speed": gps_speed,
                "gps_satellites": gps_satellites,
            }
            
            self.send_to_c_program(json.dumps(sensor_data))
            
        except Exception as e:
            self.logger.error(f"处理gps传感器数据失败: {str(e)}", exc_info=True)
            
            
    def control_callback(self, msg):
        """控制器状态数据回调函数"""
        try:
                
            # 从消息中提取传感器数据
            light = msg.light
            waterpump = msg.waterpump
            motor = msg.motor
            door = msg.door
            fan = msg.fan    
            heat = msg.heat   
            
            # 如果水泵状态发生变化（特别是关闭），则重置自动模式
            if self.control_state.waterpump != waterpump:
                if waterpump == 0:  # 水泵关闭
                    self.pump_auto_mode = False
                    self.logger.info("检测到水泵手动关闭，退出自动模式")
            
            # 更新控制状态
            self.control_state.light = light
            self.control_state.waterpump = waterpump
            self.control_state.motor = motor
            self.control_state.door = door
            self.control_state.fan = fan      
            self.control_state.heat = heat   
            
            # 添加详细日志
            self.logger.debug(f"收到控制器状态数据: light={light}, waterpump={waterpump}, motor={motor}, door={door}, fan={fan}, heat={heat}")
            
            # 创建要发送给C程序的JSON数据
            sensor_data = {
                "sensor_type": "control",
                "light": light,
                "waterpump": waterpump,
                "motor": motor,
                "door": door,
                "fan": fan,    
                "heat": heat,  
            }
            
            self.send_to_c_program(json.dumps(sensor_data))
            
        except Exception as e:
            self.logger.error(f"处理控制器状态数据失败: {str(e)}", exc_info=True)
            
    def check_thresholds_and_control(self, water_level, weight):
        """根据水位和重量阈值自动控制设备"""
        try:
            # 水位控制逻辑
            if water_level == 1:
                self.logger.warning(f"水位过低，开启水泵")
                self.control_waterpump(True, auto_mode=True)
            
            # 重量控制逻辑
            if weight < self.weight_threshold_low:
                self.logger.warning(f"重量过低 ({weight}g < {self.weight_threshold_low}g)，开启电机")
                self.control_motor(True)
            elif weight > self.weight_threshold_high:
                self.logger.warning(f"重量过高 ({weight}g > {self.weight_threshold_high}g)，关闭电机")
                self.control_motor(False)
                
        except Exception as e:
            self.logger.error(f"阈值控制失败: {str(e)}")
            
    def control_waterpump(self, state, auto_mode=False):
        """控制水泵状态"""
        try:
            # 如果已经是自动模式并且状态没有变化，则不需要操作
            if auto_mode and self.pump_auto_mode and self.control_state.waterpump == int(state):
                return
            
            # 更新控制状态
            self.control_state.waterpump = 1 if state else 0
            
            # 设置自动模式标志
            self.pump_auto_mode = auto_mode
            
            # 创建并发布控制消息
            control_msg = MultiSensorData()
            control_msg.light = self.control_state.light
            control_msg.waterpump = self.control_state.waterpump
            control_msg.motor = self.control_state.motor
            control_msg.door = self.control_state.door
            control_msg.fan = self.control_state.fan      # 风扇状态
            control_msg.heat = self.control_state.heat    # 加热器状态
            
            self.home_control_publisher.publish(control_msg)
            self.logger.info(f"已{'开启' if state else '关闭'}水泵（{'自动' if auto_mode else '手动'}模式）")
            
            # 如果开启水泵且是自动模式，则设置定时关闭
            if state and auto_mode:
                self.pump_end_time = time.time() + self.pump_duration
                self.logger.info(f"水泵将在 {self.pump_duration} 秒后自动关闭")
            
        except Exception as e:
            self.logger.error(f"控制水泵失败: {str(e)}")
            
    def control_motor(self, state):
        """控制电机状态"""
        try:
            # 如果状态没有变化，则不需要操作
            if self.control_state.motor == int(state):
                return
            
            # 更新控制状态
            self.control_state.motor = 1 if state else 0
            
            # 创建并发布控制消息
            control_msg = MultiSensorData()
            control_msg.light = self.control_state.light
            control_msg.waterpump = self.control_state.waterpump
            control_msg.motor = self.control_state.motor
            control_msg.door = self.control_state.door
            control_msg.fan = self.control_state.fan      # 风扇状态
            control_msg.heat = self.control_state.heat    # 加热器状态
            
            self.home_control_publisher.publish(control_msg)
            self.logger.info(f"已{'开启' if state else '关闭'}电机")
            
        except Exception as e:
            self.logger.error(f"控制电机失败: {str(e)}")
            
    def check_pump_status(self):
        """检查水泵状态并在需要时关闭"""
        try:
            # 只有在自动模式下才检查关闭
            if not self.pump_auto_mode:
                return
                
            # 检查是否到达关闭时间
            if self.control_state.waterpump == 1 and time.time() > self.pump_end_time:
                self.logger.info("水泵运行时间已到，自动关闭")
                self.control_waterpump(False, auto_mode=True)
                
        except Exception as e:
            self.logger.error(f"检查水泵状态失败: {str(e)}")
            
    ########################################创建传感器msg订阅#################################################

    def send_to_c_program(self, json_data):
        """发送JSON数据到C程序，确保完整消息"""
        try:
            if not self.process or self.process.stdin is None:
                self.logger.warning("C程序尚未准备好接收数据")
                return
                
            # 添加完整消息结束标记
            full_message = json_data + '\n'
            
            try:
                self.process.stdin.write(full_message)
                self.process.stdin.flush()
                self.logger.debug(f"发送完整传感器数据到C程序: {json_data}")
            except BrokenPipeError:
                self.logger.error("C程序管道已断开，无法发送传感器数据")
            except Exception as e:
                self.logger.error(f"发送传感器数据失败: {str(e)}")
                        
        except Exception as e:
            self.logger.error(f"发送数据到C程序失败: {str(e)}")

    def is_filtered_warning(self, line):
        """检查是否为需要过滤的警告"""
        for pattern in self.warning_filter:
            if pattern.search(line):
                return True
        return False

    def check_connection_status(self):
        """检查连接状态"""
        current_time = time.time()
        
        # 检查是否超时
        if current_time - self.start_time > self.connection_timeout:
            if not self.cloud_connected:
                self.logger.error("云平台连接超时！")
            elif not self.model_initialized:
                self.logger.error("物模型初始化超时！")
            self.connection_check_timer.cancel()
            return
            
        # 如果已经完成所有初始化，取消定时器
        if self.cloud_connected and self.model_initialized:
            self.connection_check_timer.cancel()
            self.logger.info("C程序初始化完成，进入正常运行状态")
            return

    def handle_stdout(self):
        """处理标准输出 - 使用行缓冲"""
        self.start_time = time.time()
        
        # 使用行缓冲读取
        for line in iter(self.process.stdout.readline, ''):
            line = line.strip()
            if not line:
                continue
                
            # 检测结构化消息
            if line.startswith("ROS2_MSG|"):
                self.process_structured_message(line)
                continue
                
            # 检测云平台连接成功消息
            if "MQTT_CONNECTED" in line:
                self.cloud_connected = True
                self.logger.info("MQTT连接成功，云平台连接已建立！")
                continue
                
            # 检测物模型初始化完成
            if "MODEL_INITIALIZED" in line:
                self.model_initialized = True
                self.logger.info("物模型初始化完成！")
                continue
                
            # 检测程序主循环开始
            if "starting main loop" in line.lower() or "进入主循环" in line.lower():
                self.logger.info("C程序已启动并进入主循环")
                continue
                
            # 其他标准输出
            self.logger.debug(f"STDOUT: {line}")
            
            # 进程结束时退出循环
            if self.process.poll() is not None:
                break

    def process_structured_message(self, line):
        """处理结构化消息 - 增强健壮性"""
        try:
            self.logger.debug(f"原始结构化消息: {line}")
            
            # 解析消息类型和内容
            parts = line.split('|')
            
            # 验证最小长度
            if len(parts) < 2:
                self.logger.error(f"结构化消息格式错误: 缺少必要字段 - {line}")
                self.message_counter['failures'] += 1
                self.message_counter['last_failure'] = line
                return
                
            # 提取消息类型 - 修复格式问题
            msg_type = None
            for part in parts:
                if part.startswith("type="):
                    msg_type = part.split('=', 1)[1]
                    break
            
            if not msg_type:
                self.logger.error(f"未找到消息类型字段: {line}")
                self.message_counter['failures'] += 1
                self.message_counter['last_failure'] = line
                return
                
            # 创建字典存储所有键值对
            data = {}
            for part in parts:
                if '=' not in part:
                    continue
                    
                key, value = part.split('=', 1)
                data[key] = value
            
            # 创建ROS2消息
            ros_msg = String()
            
            # 根据不同消息类型处理
            if msg_type == "property_set":
                # 提取msg_id
                msg_id = data.get("msg_id", "")
                
                # 尝试解析payload JSON
                payload_str = data.get("payload", "{}")
                try:
                    payload_json = json.loads(payload_str)
                    
                    # 提取特定属性值
                    motor_value = payload_json.get("home:motor", None)
                    pump_value = payload_json.get("home:pump", None)
                    light_value = payload_json.get("home:light", None)
                    door_value = payload_json.get("home:door", None)
                    fan_value = payload_json.get("home:fan", None)  
                    heat_value = payload_json.get("home:heat", None) 
                    
                    # 创建包含提取值的消息
                    extracted_data = {
                        "msg_id": msg_id,
                    }
                    
                    # 只添加解析到的属性
                    if motor_value is not None:
                        extracted_data["motor_value"] = motor_value
                    if pump_value is not None:
                        extracted_data["pump_value"] = pump_value
                    if light_value is not None:
                        extracted_data["light_value"] = light_value
                    if door_value is not None:
                        extracted_data["door_value"] = door_value
                    if fan_value is not None: 
                        extracted_data["fan_value"] = fan_value
                    if heat_value is not None:
                        extracted_data["heat_value"] = heat_value
                    
                    ros_msg.data = json.dumps(extracted_data)
                    
                    # 记录日志（只记录存在的属性）
                    log_str = "提取属性值: "
                    if motor_value is not None:
                        log_str += f"motor_value={motor_value}, "
                    if pump_value is not None:
                        log_str += f"pump_value={pump_value}, "
                    if light_value is not None:
                        log_str += f"light_value={light_value}, "
                    if door_value is not None:
                        log_str += f"door_value={door_value}, "
                    if fan_value is not None:  
                        log_str += f"fan_value={fan_value}, "
                    if heat_value is not None:
                        log_str += f"heat_value={heat_value}, "
                    self.logger.info(log_str.rstrip(', '))
                    
                    # 创建并发布控制消息（只发布解析到的值）
                    self.publish_control_data(
                        motor_value=motor_value,
                        pump_value=pump_value,
                        light_value=light_value,
                        door_value=door_value,
                        fan_value=fan_value,  
                        heat_value=heat_value  
                    )
                    
                except json.JSONDecodeError as e:
                    self.logger.error(f"无法解析payload JSON: {e}")
                    ros_msg.data = json.dumps({
                        "msg_id": msg_id,
                        "error": f"Invalid JSON: {str(e)}"
                    })
                
                self.property_set_pub.publish(ros_msg)
                self.logger.info(f"发布属性设置消息: {ros_msg.data}")
                self.message_counter['success'] += 1
                
            elif msg_type == "custom_topic":
                ros_msg.data = json.dumps({
                    "topic": data.get("topic", ""),
                    "payload": data.get("payload", "")
                })
                self.custom_topic_pub.publish(ros_msg)
                self.logger.info(f"发布自定义Topic消息: {ros_msg.data}")
                self.message_counter['success'] += 1
                
            elif msg_type == "service_call":
                ros_msg.data = json.dumps({
                    "uuid": data.get("uuid", ""),
                    "payload": data.get("payload", "")
                })
                self.service_call_pub.publish(ros_msg)
                self.logger.info(f"发布服务调用消息: {ros_msg.data}")
                self.message_counter['success'] += 1
            else:
                self.logger.warning(f"未知的消息类型: {msg_type}")
                self.message_counter['failures'] += 1
                self.message_counter['last_failure'] = line
                
        except Exception as e:
            self.logger.error(f"解析结构化消息失败: {str(e)}")
            self.logger.debug(f"导致错误的原始消息: {line}")
            self.message_counter['failures'] += 1
            self.message_counter['last_failure'] = line

    def publish_control_data(self, motor_value=None, pump_value=None, light_value=None, 
                             door_value=None, fan_value=None, heat_value=None):
        """发布控制数据到'sensors/control_data'话题，只更新提供的值"""
        try:
            # 创建控制消息
            control_msg = MultiSensorData()
            
            # 只更新提供的值，其他保持不变
            if motor_value is not None:
                self.control_state.motor = 1 if motor_value else 0
            if pump_value is not None:
                self.control_state.waterpump = 1 if pump_value else 0
                # 如果是手动设置水泵状态，退出自动模式
                if pump_value == 0:
                    self.pump_auto_mode = False
                    self.logger.info("检测到手动设置水泵状态，退出自动模式")
            if light_value is not None:
                self.control_state.light = 1 if light_value else 0
            if door_value is not None:
                self.control_state.door = 1 if door_value else 0
            if fan_value is not None:
                self.control_state.fan = 1 if fan_value else 0
            if heat_value is not None:
                self.control_state.heat = 1 if heat_value else 0
            
            # 复制当前控制状态
            control_msg.motor = self.control_state.motor
            control_msg.waterpump = self.control_state.waterpump
            control_msg.light = self.control_state.light
            control_msg.door = self.control_state.door
            control_msg.fan = self.control_state.fan
            control_msg.heat = self.control_state.heat
            
            # 发布控制消息
            self.home_control_publisher.publish(control_msg)
            
            # 记录日志（只记录变化的属性）
            log_str = "发布控制指令: "
            if motor_value is not None:
                log_str += f"motor={control_msg.motor}, "
            if pump_value is not None:
                log_str += f"waterpump={control_msg.waterpump}, "
            if light_value is not None:
                log_str += f"light={control_msg.light}, "
            if door_value is not None:
                log_str += f"door={control_msg.door}, "
            if fan_value is not None:  
                log_str += f"fan={control_msg.fan}, "
            if heat_value is not None:  
                log_str += f"heat={control_msg.heat}, "
            self.logger.info(log_str.rstrip(', '))
            
        except Exception as e:
            self.logger.error(f"发布控制数据失败: {str(e)}")

    def handle_stderr(self):
        """处理标准错误 - 过滤特定警告"""
        for line in iter(self.process.stderr.readline, ''):
            line = line.strip()
            if not line:
                continue
                
            # 过滤特定警告
            if self.is_filtered_warning(line):
                current_time = time.time()
                
                # 限制警告输出频率：每10秒输出一次
                if current_time - self.last_warning_time > 10:
                    self.warning_count += 1
                    self.logger.warn(f"过滤警告[{self.warning_count}]: {line.split(']')[-1].strip()}")
                    self.last_warning_time = current_time
                continue
                
            # 检测连接失败消息
            if "onesdk_connect failed" in line.lower():
                self.logger.error("云平台连接失败！")
                continue
                
            if "connection failed" in line.lower():
                self.logger.error("连接失败！")
                continue
                
            # MQTT连接错误
            if "mqtt error" in line.lower() or "mqtt failed" in line.lower():
                self.logger.error(f"MQTT错误: {line}")
                continue
                
            # 认证错误
            if "authentication failed" in line.lower() or "auth error" in line.lower():
                self.logger.error(f"认证失败: {line}")
                continue
                
            # 其他错误输出
            self.logger.error(f"STDERR: {line}")
            
            # 进程结束时退出循环
            if self.process.poll() is not None:
                break

    def destroy_node(self):
        """节点销毁时终止进程"""
        self.logger.info("正在关闭，终止C程序...")
        
        # 关闭所有设备
        self.control_fan(False)
        self.control_heater(False)
        
        if self.process.poll() is not None:
            exit_code = self.process.returncode
            self.logger.error(f"C程序退出，代码: {exit_code}")
        
        # 输出消息统计
        total = self.message_counter['success'] + self.message_counter['failures']
        success_rate = (self.message_counter['success'] / total * 100) if total > 0 else 0
        self.logger.info(f"消息处理统计: 成功={self.message_counter['success']}, 失败={self.message_counter['failures']}, 成功率={success_rate:.1f}%")
        if self.message_counter['failures'] > 0:
            self.logger.warning(f"最后一条失败消息: {self.message_counter['last_failure']}")
        
        if hasattr(self, 'process') and self.process:
            # 取消连接状态检查定时器
            if hasattr(self, 'connection_check_timer'):
                self.connection_check_timer.cancel()
            
            # 关闭标准输入管道
            if self.process.stdin:
                try:
                    self.process.stdin.close()
                except Exception as e:
                    self.logger.error(f"关闭标准输入失败: {str(e)}")
            
            # 发送信号到整个进程组
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
                
                # 等待进程结束
                try:
                    self.process.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    self.logger.warning("进程未终止，发送SIGKILL")
                    os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
                    self.process.wait()
            except ProcessLookupError:
                pass
            
            self.process = None
        
        # 输出最终警告统计
        if self.warning_count > 0:
            self.logger.warn(f"总共过滤了 {self.warning_count} 条队列深度警告")
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = IoTBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()