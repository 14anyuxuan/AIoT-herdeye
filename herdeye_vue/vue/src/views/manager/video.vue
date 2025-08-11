<template>
  <div class="pasture-eye-page">
    <!-- 页面标题区域 -->
    <header class="page-header">
      <h1 class="page-title">牧瞳系统</h1>
      <div class="page-meta">
        <span class="update-time">上次更新：{{ updateTime }}</span>
      </div>
    </header>

    <!-- 检测功能模块 -->
    <el-card class="detection-card">
      <!-- 视频流容器 - 并排显示 -->
      <div class="videos-container">
        <!-- 无人机视频流 -->
        <div class="video-module">
          <div class="detection-header">
            <span class="video-title">无人机视频：牲畜数量实时统计</span>
            <el-button
              :type="streams.drone.isStreaming ? 'danger' : 'success'"
              @click="toggleStream('drone')"
              class="control-button"
            >
              {{ streams.drone.isStreaming ? '关闭视频' : '开启视频' }}
            </el-button>
          </div>

          <div class="video-container">
            <img
              :src="streams.drone.src"
              class="video-stream"
              v-show="streams.drone.isStreaming"
              alt="无人机视频流"
              :class="{ 'video-loading': streams.drone.isLoading }"
            />
            <div v-show="!streams.drone.isStreaming && streams.drone.latestUrl" class="video-links">
              <h3>无人机录制视频</h3>
              <el-link :href="streams.drone.latestUrl" target="_blank" type="primary" class="download-link">
                下载最新视频 <i class="el-icon-download"></i>
              </el-link>
            </div>
            <div v-show="!streams.drone.isStreaming && !streams.drone.latestUrl" class="video-placeholder">
              <i class="el-icon-video-camera"></i>
              <p>点击开启无人机视频流</p>
            </div>
          </div>
        </div>

        <!-- 地面端视频流 -->
        <div class="video-module">
          <div class="detection-header">
            <span class="video-title">地面端视频：智能车画面实时跟踪</span>
            <el-button
              :type="streams.ground.isStreaming ? 'danger' : 'success'"
              @click="toggleStream('ground')"
              class="control-button"
            >
              {{ streams.ground.isStreaming ? '关闭视频' : '开启视频' }}
            </el-button>
          </div>

          <div class="video-container">
            <img
              :src="streams.ground.src"
              class="video-stream"
              v-show="streams.ground.isStreaming"
              alt="地面端视频流"
              :class="{ 'video-loading': streams.ground.isLoading }"
            />
            <div v-show="!streams.ground.isStreaming && streams.ground.latestUrl" class="video-links">
              <h3>地面端录制视频</h3>
              <el-link :href="streams.ground.latestUrl" target="_blank" type="primary" class="download-link">
                下载最新视频 <i class="el-icon-download"></i>
              </el-link>
            </div>
            <div v-show="!streams.ground.isStreaming && !streams.ground.latestUrl" class="video-placeholder">
              <i class="el-icon-video-camera"></i>
              <p>点击开启地面端视频流</p>
            </div>
          </div>
        </div>
      </div>
    </el-card>
  </div>
</template>

<script>
import axios from "axios";

export default {
  name: 'Home',
  data() {
    return {
      updateTime: new Date().toLocaleString(),
      streams: { 
        drone: {
          isStreaming: false,
          src: '',
          latestUrl: '',
          isLoading: false,
          intervals: { refresh: null, detection: null }
        },
        ground: {
          isStreaming: false,
          src: '',
          latestUrl: '',
          isLoading: false,
          intervals: { refresh: null, detection: null }
        }
      }
    };
  },
  methods: {
    // 跳转GPS页面
    navigateToGPS() {
      this.$router.push('/GPS');
    },

    // 切换视频流状态
    toggleStream(type) {
      if (this.streams[type].isStreaming) {
        this.stopStream(type);
      } else {
        this.startStream(type);
      }
    },

    // 启动视频流
    startStream(type) {
      const stream = this.streams[type];
      stream.isStreaming = true;
      stream.isLoading = true; // 显示加载状态
      stream.src = `http://localhost:5000/video_feed?type=${type}&t=${Date.now()}`;
      
      // 清除旧定时器
      Object.values(stream.intervals).forEach(interval => interval && clearInterval(interval));
      
      // 模拟加载完成
      setTimeout(() => {
        stream.isLoading = false;
      }, 1000);
    },

    // 停止视频流
    async stopStream(type) {
      const stream = this.streams[type];
      stream.isStreaming = false;
      stream.src = '';
      
      // 清除检测轮询
      clearInterval(stream.intervals.detection);
      
      // 获取最新视频并启动刷新轮询
      await this.fetchLatestVideo(type);
      stream.intervals.refresh = setInterval(() => !stream.isStreaming && this.fetchLatestVideo(type), 5000);
    },

    // 获取最新录制视频
    async fetchLatestVideo(type) {
      try {
        const res = await this.$request.get(`http://localhost:5000/latest_video?type=${type}`);
        this.streams[type].latestUrl = res.url;
      } catch (error) {
        console.error(`${type === 'drone' ? '无人机' : '地面端'}视频获取失败`, error);
      }
    }
  },
  // 清理定时器
  beforeUnmount() {
    Object.values(this.streams).forEach(stream => {
      Object.values(stream.intervals).forEach(interval => interval && clearInterval(interval));
    });
  },
  // 初始化时间更新
  created() {
    setInterval(() => this.updateTime = new Date().toLocaleString(), 60000);
  }
};
</script>

<style scoped>
.pasture-eye-page {
  padding: 24px;
  background-color: #f5f7fa;
  min-height: 100vh;
  box-sizing: border-box;
}

.page-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 24px;
  padding-bottom: 16px;
  border-bottom: 1px solid #e5e6eb;
}

.page-title {
  margin: 0;
  font-size: 22px;
  font-weight: 600;
  color: #1d2129;
}

.page-meta {
  color: #86909c;
  font-size: 14px;
  display: flex;
  align-items: center;
}

.page-meta .update-time::before {
  content: "";
  display: inline-block;
  width: 16px;
  height: 16px;
  background: url("data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' width='16' height='16' viewBox='0 0 24 24' fill='none' stroke='%2386909c' stroke-width='2' stroke-linecap='round' stroke-linejoin='round'%3E%3Ccircle cx='12' cy='12' r='10'%3E%3C/circle%3E%3Cpolyline points='12 6 12 12 16 14'%3E%3C/polyline%3E%3C/svg%3E") no-repeat center;
  margin-right: 6px;
  vertical-align: middle;
}

.detection-card {
  padding: 24px;
  border-radius: 8px;
  box-shadow: 0 2px 8px rgba(0, 0, 0, 0.08);
  border: none;
  background-color: #fff;
}

.videos-container {
  display: flex;
  gap: 24px;
  width: 100%;
}

.video-module {
  flex: 1;
  margin-bottom: 0;
  padding-bottom: 0;
  transition: all 0.3s ease;
}

.detection-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 16px;
  padding-bottom: 12px;
  border-bottom: 1px solid #f2f3f5;
}

.video-title {
  font-size: 16px;
  font-weight: 500;
  color: #1d2129;
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
}

.control-button {
  padding: 6px 16px;
  border-radius: 4px;
  transition: all 0.2s ease;
}

.control-button:hover {
  transform: translateY(-1px);
  box-shadow: 0 2px 5px rgba(0, 0, 0, 0.1);
}

.video-container {
  border: 1px solid #e5e6eb;
  border-radius: 6px;
  padding: 16px;
  background-color: #fafafa;
  margin-bottom: 0;
  min-height: 450px;
  display: flex;
  flex-direction: column;
  justify-content: center;
  align-items: center;
  position: relative;
  transition: border-color 0.3s ease;
}

.video-container:hover {
  border-color: #c9cdD4;
}

.video-stream {
  max-width: 100%;
  height: auto;
  min-height: 400px;
  object-fit: contain;
  border-radius: 4px;
  transition: all 0.3s ease;
}

.video-loading {
  animation: pulse 1.5s infinite;
}

@keyframes pulse {
  0% {
    opacity: 1;
  }
  50% {
    opacity: 0.7;
  }
  100% {
    opacity: 1;
  }
}

.video-links {
  text-align: center;
  padding: 30px 20px;
  color: #4e5969;
}

.video-links h3 {
  margin-bottom: 16px;
  font-size: 16px;
  font-weight: 500;
  color: #1d2129;
}

.download-link {
  font-size: 14px;
  padding: 6px 12px;
  border-radius: 4px;
  transition: all 0.2s ease;
}

.download-link:hover {
  background-color: #f0f7ff;
}

.video-placeholder {
  text-align: center;
  color: #86909c;
  padding: 30px 20px;
}

.video-placeholder i {
  font-size: 48px;
  margin-bottom: 16px;
  color: #c9cdD4;
  transition: color 0.3s ease;
}

.video-module:hover .video-placeholder i {
  color: #86909c;
}

.video-placeholder p {
  margin: 0;
  font-size: 14px;
}

/* 响应式调整 */
@media (max-width: 1024px) {
  .videos-container {
    gap: 16px;
  }
  
  .video-container {
    min-height: 380px;
  }
  
  .video-stream {
    min-height: 340px;
  }
}

@media (max-width: 768px) {
  .pasture-eye-page {
    padding: 16px;
  }
  
  .page-header {
    flex-direction: column;
    align-items: flex-start;
    gap: 12px;
    margin-bottom: 16px;
    padding-bottom: 12px;
  }
  
  .videos-container {
    flex-direction: column;
    gap: 24px;
  }
  
  .video-module {
    width: 100%;
  }
  
  .detection-card {
    padding: 16px;
  }
  
  .detection-header {
    flex-direction: column;
    align-items: flex-start;
    gap: 12px;
    margin-bottom: 12px;
    padding-bottom: 8px;
  }
  
  .video-title {
    white-space: normal;
    line-height: 1.5;
  }
  
  .video-container {
    min-height: 280px;
    padding: 12px;
  }
  
  .video-stream {
    min-height: 240px;
  }
}

@media (max-width: 480px) {
  .page-title {
    font-size: 18px;
  }
  
  .video-title {
    font-size: 14px;
  }
  
  .control-button {
    width: 100%;
    margin-left: 0 !important;
  }
}
</style>