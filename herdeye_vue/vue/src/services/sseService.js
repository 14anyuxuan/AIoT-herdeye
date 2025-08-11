import { ref } from 'vue'

// 创建事件总线（优化内存泄漏）
const eventBus = {
  events: {},
  on(event, callback) {
    if (!this.events[event]) this.events[event] = [];
    this.events[event].push(callback);
  },
  emit(event, data) {
    if (this.events[event]) {
      // 复制一份回调列表，避免执行中删除导致的问题
      [...this.events[event]].forEach(callback => callback(data));
    }
  },
  off(event, callback) {
    if (this.events[event]) {
      this.events[event] = this.events[event].filter(cb => cb !== callback);
    }
  }
};

// SSE连接配置
const backendUrl = "ws://8.152.103.136:9090/ws/data";
let mainEs = null;
let mapEs = null;
let isConnected = ref(false);

// 创建SSE连接的通用方法（优化连接管理）
const createSSE = (url, eventName) => {
  // 关闭已存在的连接并清空引用
  if (eventName === 'main' && mainEs) {
    mainEs.close();
    mainEs = null;
  }
  if (eventName === 'map' && mapEs) {
    mapEs.close();
    mapEs = null;
  }

  const ws = new WebSocket(url);

  // 连接状态检查函数
  const updateConnectionStatus = () => {
    const isOpen = ws.readyState === WebSocket.OPEN;
    isConnected.value = isOpen;
    eventBus.emit('connectionStatus', isOpen);
  };

  // 初始连接成功
  ws.onopen = () => {
    updateConnectionStatus();
    console.log(`${eventName} WebSocket连接已建立`);
  };

  // 接收消息处理
  ws.onmessage = (ev) => {
    try {
        // 打印原始接收数据
        console.log(`[${new Date().toISOString()}] ${eventName} WebSocket接收到原始数据:`, ev.data);
        
        // 解析JSON数据
        const data = JSON.parse(ev.data);
        console.log(`[${new Date().toISOString()}] ${eventName} WebSocket数据解析成功:`, data);
        
        // 可以单独提取各个字段
        const temperature = data.T;       // 温度: 30.29
        const humidity = data.H;          // 湿度: 64.26
        const longitude = data.lng;       // 经度: 0.000000
        const latitude = data.lat;        // 纬度: 0.000000
        const altitude = data.altitude;   // 海拔: 0.000000
        const course = data.cogt;         // 航向: 0.000000
        const speed = data.speed;         // 速度: 0.00
        const waterLevel = data.water_level; // 水位: 1
        const weight = data.weight;       // 重量: 45
        const pumpStatus = data.pump;     // 泵状态: 1 (可能表示开启)
        const motorStatus = data.motor;   // 电机状态: 0 (可能表示关闭)
        const lightStatus = data.light;   // 灯光状态: 0 (可能表示关闭)
        const doorStatus = data.door;     // 门状态: 0 (可能表示关闭)
        const heatStatus = data.heat;     // 加热状态: 0 (可能表示关闭)
        const fanStatus = data.fan;       // 风扇状态: 1 (可能表示开启)

        const postData = {
            time: new Date().toISOString(), // 添加当前时间戳
            t: temperature,
            h: humidity,
            lng: longitude,
            lat: latitude,
            altitude: altitude,
            cogt: course,
            speed: speed,
            waterlevel: waterLevel,
            weight: weight,
            pump: pumpStatus,
            motor: motorStatus,
            light: lightStatus,
            door: doorStatus,
            heat: heatStatus,
            fan: fanStatus
        };
        
        // 发送add请求到后端
        console.log(`[${new Date().toISOString()}] 准备发送数据到后端:`, postData);
        fetch('http://8.152.103.136:9090/rtsd/add', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
                // 可以添加其他需要的请求头，如认证信息
                // 'Authorization': 'Bearer ' + token
            },
            body: JSON.stringify(postData)
        })
        .then(response => {
            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }
            return response.json();
        })
        .then(result => {
            console.log(`[${new Date().toISOString()}] 数据添加成功:`, result);
        })
        .catch(error => {
            console.error(`[${new Date().toISOString()}] 数据添加失败:`, error);
        });   

        // 触发事件，传递解析后的数据
        eventBus.emit(eventName, data);
        console.log(`[${new Date().toISOString()}] ${eventName} 事件已触发，数据已传递`);
        
    } catch (e) {
        console.error(`[${new Date().toISOString()}] ${eventName} WebSocket数据解析失败:`, e);
        console.error(`[${new Date().toISOString()}] 解析失败的原始数据:`, ev.data);
    }
  };

  // 错误处理与重连
  ws.onerror = () => {
    updateConnectionStatus();
    console.error(`${eventName} WebSocket连接错误，尝试重连...`);
    // 避免重复重连（仅当前连接实例未被替换时）
    if ((eventName === 'main' && mainEs === ws) || (eventName === 'map' && mapEs === ws)) {
      setTimeout(() => createSSE(url, eventName), 5000);
    }
  };

  // 后端主动关闭连接时重连
  ws.onclose = () => {
    updateConnectionStatus();
    console.log(`${eventName} WebSocket连接被关闭，尝试重连...`);
    if ((eventName === 'main' && mainEs === ws) || (eventName === 'map' && mapEs === ws)) {
      setTimeout(() => createSSE(url, eventName), 5000);
    }
  };

  // 保存连接实例
  if (eventName === 'main') mainEs = ws;
  if (eventName === 'map') mapEs = ws;

  return ws;
};

// 初始化连接（使用区分的端点）
const initConnections = () => {
  createSSE(backendUrl, 'main');    // 主数据端点
  createSSE(backendUrl, 'map');    // 地图数据端点
};

// 关闭所有连接（优化引用清理）
const closeConnections = () => {
  if (mainEs) {
    mainEs.close();
    mainEs = null;
  }
  if (mapEs) {
    mapEs.close();
    mapEs = null;
  }
  isConnected.value = false;
  eventBus.emit('connectionStatus', false);
};

// 导出服务（优化事件解绑）
export default {
  eventBus,
  isConnected,
  initConnections,
  closeConnections,
  onMainData: (callback) => {
    eventBus.on('main', callback);
    return () => eventBus.off('main', callback); // 返回解绑函数
  },
  onMapData: (callback) => {
    eventBus.on('map', callback);
    return () => eventBus.off('map', callback);
  },
  onConnectionStatus: (callback) => {
    eventBus.on('connectionStatus', callback);
    return () => eventBus.off('connectionStatus', callback);
  }
};