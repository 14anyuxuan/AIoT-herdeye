# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from herdeye_volcllm_srv.srv import LLMRequest
from openai import OpenAI

import base64
import hashlib
import hmac
import random
import time
import platform
import requests
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
from cryptography.hazmat.backends import default_backend


class VolcOneDevice:
    DYNAMIC_REGISTER_PATH = "/2021-12-14/DynamicRegister"
    GET_LLM_CONFIG_PATH = "/2021-12-14/GetLLMConfig"
    API_VERSION = "2021-12-14"
    API_VERSION_QUERY_PARAM = "Version=2021-12-14"
    API_ACTION_DYNAMIC_REGISTER = "Action=DynamicRegister"
    API_ACTION_GET_LLM_CONFIG =  "Action=GetLLMConfig"


    # aigw header
    HEADER_SIGNATURE = "X-Signature"
    HEADER_AUTH_TYPE = "X-Auth-Type"
    HEADER_DEVICE_NAME = "X-Device-Name"
    HEADER_PRODUCT_KEY = "X-Product-Key"
    HEADER_RANDOM_NUM = "X-Random-Num"
    HEADER_TIMESTAMP = "X-Timestamp"
    HEADER_HARDWARE_ID = "X-Hardware-Id"

    # 一机一密，需要提供ProductKey、DeviceName、DeviceSecret
    IOT_AUTH_DEVICE_SECRET = -1
    # 一型一密预注册，需要提供ProductKey、ProductSecret、DeviceName
    IOT_AUTH_DYNAMIC_PRE_REGISTERED = 0
    # 一型一密免预注册，需要提供ProductKey、ProductSecret、Name
    IOT_AUTH_DYNAMIC_NO_PRE_REGISTERED = 1
    IOT_AUTH_UNKNOWN = 100

    def __init__(self, auth_type, http_host, instance_id, device_name, product_key,
                 product_secret=None, device_secret=None, use_ssl=True):
        self.auth_type = auth_type
        self.http_host = http_host
        self.instance_id = instance_id
        self.device_name = device_name
        self.device_secret = device_secret
        self.product_key = product_key
        self.product_secret = product_secret
        self.use_ssl = use_ssl
        self._persistent_mac = ""

    def dynamic_register(self):
        if self.device_secret:
            return self.device_secret

        rnd = random.randint(0, 0xFFFFFFFF) # 32bit random int
        ts = int(time.time())

        signature = self.hmac_256_encrypt(
            auth_type=self.auth_type,
            device_name=self.device_name,
            random_num=rnd,
            product_key=self.product_key,
            timestamp=ts,
            secret=self.product_secret
        )
        post_data = {
            "InstanceID": self.instance_id,
            "product_key": self.product_key,
            "device_name": self.device_name,
            "random_num": rnd,
            "timestamp": ts,
            "auth_type": self.auth_type,
            "signature": signature,
        }

        protocol = "https" if self.use_ssl else "http"
        url = f"{protocol}://{self.http_host}{self.DYNAMIC_REGISTER_PATH}?{self.API_ACTION_DYNAMIC_REGISTER}&{self.API_VERSION_QUERY_PARAM}"
        headers = {
            'Content-Type': 'application/json'
        }
        # use http post
        resp = requests.post(url, json=post_data, headers=headers)
        if resp.status_code != 200:
            raise Exception(f"dynamic register failed, status code: {resp.status_code}, resp: {resp.text}")
        resp_json = resp.json()
        if resp_json["ResponseMetadata"]["Action"] != "DynamicRegister":
            raise Exception(f"dynamic register failed, resp: {resp.text}")
        # 使用 get 方法安全获取 Error 键的值
        error = resp_json["ResponseMetadata"].get("Error")
        if error is not None:
            raise Exception(f"dynamic register failed, resp: {resp.text}")
        encryped_device_secret = resp_json["Result"]["payload"]
        self.device_secret = self.aes_decode(self.product_secret, encryped_device_secret, partial_secret=True)

        return self.device_secret

    def aigw_api_key(self):
        rnd = random.randint(0, 0xFFFFFFFF)  # 32bit random int
        ts = int(time.time())
        secret = self.dynamic_register()

        signature = self.hmac_256_encrypt(
            auth_type=0, # must be 0
            device_name=self.device_name,
            random_num=rnd,
            product_key=self.product_key,
            timestamp=ts,
            secret=secret
        )
        post_data = {
            "InstanceID": self.instance_id,
            "product_key": self.product_key,
            "device_name": self.device_name,
            "random_num": rnd,
            "timestamp": ts,
            "auth_type": 0, # must be 0
            "signature": signature,
        }

        protocol = "https" if self.use_ssl else "http"
        url = f"{protocol}://{self.http_host}{self.GET_LLM_CONFIG_PATH}?{self.API_ACTION_GET_LLM_CONFIG}&{self.API_VERSION_QUERY_PARAM}"
        headers = {
            'Content-Type': 'application/json'
        }
        # use http post
        resp = requests.post(url, json=post_data, headers=headers)
        if resp.status_code != 200:
            raise Exception(f"get llm config failed, status code: {resp.status_code}, resp: {resp.text}")
        resp_json = resp.json()
        if resp_json["ResponseMetadata"]["Action"] != "GetLLMConfig":
            raise Exception(f"get llm config failed, resp: {resp.text}")
        # 使用 get 方法安全获取 Error 键的值
        error = resp_json["ResponseMetadata"].get("Error")
        if error is not None:
            raise Exception(f"get llm config failed, resp: {resp.text}")
        encryped_key = resp_json["Result"]["APIKey"]
        api_key = self.aes_decode(secret, encryped_key, partial_secret=False)

        gateway_url = resp_json["Result"]["URL"]
        return gateway_url+"/v1", api_key

    def aigw_auth_headers(self):
        if not self.device_secret:
            self.dynamic_register()
        rnd = random.randint(0, 0xFFFFFFFF)  # 32bit random int
        ts = int(time.time())

        signature = self.hmac_256_encrypt(
            auth_type=self.auth_type,
            device_name=self.device_name,
            random_num=rnd,
            product_key=self.product_key,
            timestamp=ts,
            secret=self.device_secret
        )
        headers = {
            self.HEADER_SIGNATURE: signature,
            self.HEADER_AUTH_TYPE: str(self.auth_type),
            self.HEADER_DEVICE_NAME: self.device_name,
            self.HEADER_PRODUCT_KEY: self.product_key,
            self.HEADER_RANDOM_NUM: str(rnd),
            self.HEADER_TIMESTAMP: str(ts),
            self.HEADER_HARDWARE_ID: self.hardware_id(),
        }
        return headers

    @staticmethod
    def hmac_256_encrypt(auth_type, device_name, random_num, product_key, timestamp, secret):
        content = f"auth_type={auth_type}&device_name={device_name}&random_num={random_num}&product_key={product_key}&timestamp={timestamp}"
        # 生成HMAC-SHA256签名
        key = secret.encode('utf-8')
        h = hmac.new(key, digestmod=hashlib.sha256)
        h.update(content.encode('utf-8'))

        sig = base64.b64encode(h.digest()).decode('utf-8')
        return sig

    @staticmethod
    def pkcs5_unpadding(data):
        """
        去除PKCS#5填充。
        :param data: 带填充的明文字节
        :return: 去除填充后的明文字节
        """
        if len(data) == 0:
            return data
        padding = data[-1]
        if padding > len(data):
            raise ValueError("Invalid padding")
        return data[:-padding]

    @staticmethod
    def aes_decode(device_secret, encrypt_data, partial_secret=False):
        """
        解密函数，使用AES-CBC模式，去除URL安全字符。
        :param device_secret: 设备密钥字符串
        :param encrypt_data: Base64编码的密文字符串
        :param partial_secret: 是否是部分密钥
        :return: 解密后的明文字符串，或错误
        """
        try:
            # 解码Base64
            decoded_data = base64.b64decode(encrypt_data)
            # 处理密钥和IV
            if partial_secret:
                key = device_secret[:16].encode('utf-8')
                iv = device_secret[:16].encode('utf-8')
            else:
                key = device_secret.encode('utf-8')
                iv = device_secret[:16].encode('utf-8')

            # 创建AES解密器
            backend = default_backend()
            cipher = Cipher(algorithms.AES(key), modes.CBC(iv), backend=backend)
            decryptor = cipher.decryptor()

            # 解密
            decrypted = decryptor.update(decoded_data) + decryptor.finalize()

            # 去除PKCS#5填充
            decrypted = VolcOneDevice.pkcs5_unpadding(decrypted)

            return decrypted.decode('utf-8')
        except Exception as e:
            print(f"解密失败: {e}")
            return None

    def hardware_id(self):
        """
        获取当前系统默认网卡的MAC地址。
        如果不是Linux系统，返回 fallback_mac_addr 生成的地址。
        """
        if platform.system() == "Linux":
            import socket
            import fcntl
            import struct

            try:
                # 获取默认网关的接口
                found = False
                with open('/proc/net/route') as f:
                    for line in f:
                        fields = line.strip().split()
                        if fields[1] == '00000000':  # 默认网关
                            interface = fields[0]
                            found = True
                            break

                if not found:
                   return self.fallback_mac_addr()
                # 获取该接口的MAC地址
                s = socket.socket(socket.AF_PACKET, socket.SOCK_RAW)
                info = fcntl.ioctl(s.fileno(), 0x8927, struct.pack('256s', bytes(interface, 'utf-8')[:15]))
                return ':'.join('%02x' % b for b in info[18:24])
            except Exception as e:
                print(f"Please run this script as the root user to ensure proper functionality.")

        return self.fallback_mac_addr()

    def fallback_mac_addr(self):
        """
        生成一个符合规范的MAC地址（本地管理地址 + 单播）。
        仅首次调用时生成，后续调用返回之前生成的地址。
        """
        if self._persistent_mac:
            return self._persistent_mac
        now = time.time()
        tm_info = time.localtime(now)
        hour_seed = tm_info.tm_hour * 3600
        random.seed(hour_seed)
        bytes_list = [0] * 6

        # 生成符合规范的MAC地址（本地管理地址 + 单播）
        bytes_list[0] = (random.randint(0, 0xFE) | 0x02)  # 保证是本地管理地址
        for i in range(1, 6):
            bytes_list[i] = random.randint(0, 0xFF)

        # 格式化为MAC地址字符串
        self._persistent_mac = ':'.join('{:02x}'.format(byte) for byte in bytes_list)

        return self._persistent_mac





class LLMServer(Node):
    def __init__(self):
        super().__init__('volc_llm_server')
        self.srv = self.create_service(LLMRequest, 'volc_llm', self.handle_request)
        
        # 火山引擎认证配置
        self.http_host = "iot-cn-shanghai.iot.volces.com"
        self.instance_id = "684a6fed9541492e944b6a76"
        self.device_name = "Herdeye_car"
        self.product_key = "68510a81a15eeaadc5ea02ae"
        self.product_secret = "a6aad77a189bae059efe42b1"
        self.model_id = "7524302269345660938"
        
        # 初始化设备认证
        self.dev = VolcOneDevice(
            auth_type=VolcOneDevice.IOT_AUTH_DYNAMIC_NO_PRE_REGISTERED,
            http_host=self.http_host,
            instance_id=self.instance_id,
            device_name=self.device_name,
            product_key=self.product_key,
            product_secret=self.product_secret,
            use_ssl=True,
        )
        
        # 获取API凭证
        api_url, api_key = self.dev.aigw_api_key()
        self.get_logger().info(f"API URL: {api_url}")
        
        # 创建OpenAI客户端
        self.client = OpenAI(
            base_url=api_url,
            api_key=api_key,
        )
        
        self.get_logger().info("LLM Server 准备就绪")

    def handle_request(self, request, response):
        self.get_logger().info(f"收到请求: {request.request}")
        
        try:
            # 生成新的认证头部
            headers = self.dev.aigw_auth_headers()
            
            # 创建对话历史
            messages = [
                {"role": "system", "content": "You are a helpful assistant."},
                {"role": "user", "content": request.request}
            ]
            
            # 发送请求到模型
            model_response = self.client.chat.completions.create(
                model=self.model_id,
                messages=messages,
                stream=False,  # 使用非流式响应
                extra_headers=headers
            )
            
            # 获取完整回复
            reply = model_response.choices[0].message.content
            response.response = reply
            self.get_logger().info(f"模型回复: {reply}")
            
        except Exception as e:
            self.get_logger().error(f"请求失败: {str(e)}")
            response.response = f"错误: {str(e)}"
            
        return response

def main(args=None):
    rclpy.init(args=args)
    server = LLMServer()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()