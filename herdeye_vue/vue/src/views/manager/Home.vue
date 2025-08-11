<template>
  <div>
    <div class="card" style="padding: 15px">
      您好，{{ user?.name }}！欢迎使用本系统
    </div>
  </div>
</template>

<script>

export default {
  name: 'Home',
  data() {
    return {
      user: JSON.parse(localStorage.getItem('xm-user') || '{}'),
      notices: [],
      // 新增WebSocket和Worker相关状态
      worker: null,
      workerPort: null,
      connectionStatus: false
    }
  },
  created() {
  },
  mounted() {
    this.initWorker();
  },
  methods: {
    // 新增: 初始化SharedWorker
    initWorker() {
      if (!window.SharedWorker) {
        console.error('当前浏览器不支持SharedWorker');
        return;
      }

      try {
        this.worker = new SharedWorker('/websocket-shared-worker.js');
        this.workerPort = this.worker.port;

        // 监听Worker消息
        this.workerPort.onmessage = (event) => {
          this.handleWorkerMessage(event.data);
        };

        // 监听错误
        this.workerPort.onerror = (error) => {
          console.error('Worker error:', error);
          this.connectionStatus = false;
        };

        // 启动连接
        this.workerPort.start();
      } catch (error) {
        console.error('初始化SharedWorker失败:', error);
      }
    },

    // 新增: 处理Worker消息
    handleWorkerMessage(message) {
      switch (message.type) {
        case 'CONNECTED':
          this.connectionStatus = true;
          console.log('WebSocket连接已建立');
          break;
        case 'DISCONNECTED':
          this.connectionStatus = false;
          console.log('WebSocket连接已断开');
          break;
        case 'ERROR':
          console.error('WebSocket错误:', message.data);
          this.connectionStatus = false;
          break;
        // 可以根据需要添加其他消息类型的处理
        default:
          console.log('未知消息类型:', message.type);
      }
    }
  },
  beforeDestroy() {
    // 清理Worker连接
    if (this.workerPort) {
      this.workerPort.close();
      this.worker = null;
      this.workerPort = null;
    }
  }
}
</script>