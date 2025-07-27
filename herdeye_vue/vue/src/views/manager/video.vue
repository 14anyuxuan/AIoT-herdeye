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
      <!-- 无人机视频流 -->
      <div class="video-module">
        <div class="detection-header">
          <el-select v-model="models.drone" @change="handleModelChange('drone')" placeholder="请选择模型">
            <el-option label="无人机检测模型" value="drone"></el-option>
          </el-select>
          <el-button
            :type="streams.drone.isStreaming ? 'danger' : 'success'"
            @click="toggleStream('drone')"
            style="margin-left: 10px"
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
          />
          <div v-show="!streams.drone.isStreaming && streams.drone.latestUrl" class="video-links">
            <h3>无人机录制视频</h3>
            <el-link :href="streams.drone.latestUrl" target="_blank" type="primary">
              下载最新视频 <i class="el-icon-download"></i>
            </el-link>
          </div>
        </div>

        <!-- 无人机检测结果 -->
        <div class="realtime-results">
          <h3>无人机实时检测结果</h3>
          <el-table
            :data="streams.drone.detections"
            stripe
            height="300"
            style="width: 100%"
            v-loading="streams.drone.isLoading"
          >
            <el-table-column prop="label" label="目标类型" width="120">
              <template #default="{row}"><el-tag type="info">{{ row.label }}</el-tag></template>
            </el-table-column>
            <el-table-column label="置信度" width="120">
              <template #default="{row}">
                <el-progress
                  :percentage="row.confidence * 100"
                  :format="formatConfidence"
                  :color="getConfidenceColor"
                  stroke-width="16"
                  status="success"
                />
              </template>
            </el-table-column>
            <el-table-column label="位置坐标">
              <template #default="{row}">
                <div class="coordinates">
                  <span class="coord-item">X: {{ row.x.toFixed(0) }}</span>
                  <span class="coord-item">Y: {{ row.y.toFixed(0) }}</span>
                </div>
              </template>
            </el-table-column>
            <el-table-column label="原始位置" width="180">
              <template #default="{row}">
                <div class="raw-coords">
                  <div>xmin: {{ row.xmin }}</div>
                  <div>ymin: {{ row.ymin }}</div>
                  <div>xmax: {{ row.xmax }}</div>
                  <div>ymax: {{ row.ymax }}</div>
                </div>
              </template>
            </el-table-column>
            <template #empty>
              <div class="empty-status">
                <i class="el-icon-video-camera-solid"></i>
                <p>{{ streams.drone.isStreaming ? '暂无检测数据' : '请开启视频流获取结果' }}</p>
              </div>
            </template>
          </el-table>
        </div>
      </div>

      <!-- 分隔线 -->
      <div class="video-separator"></div>

      <!-- 地面端视频流 -->
      <div class="video-module">
        <div class="detection-header">
          <el-select v-model="models.ground" @change="handleModelChange('ground')" placeholder="请选择模型">
            <el-option label="地面端检测模型" value="ground"></el-option>
          </el-select>
          <el-button
            :type="streams.ground.isStreaming ? 'danger' : 'success'"
            @click="toggleStream('ground')"
            style="margin-left: 10px"
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
          />
          <div v-show="!streams.ground.isStreaming && streams.ground.latestUrl" class="video-links">
            <h3>地面端录制视频</h3>
            <el-link :href="streams.ground.latestUrl" target="_blank" type="primary">
              下载最新视频 <i class="el-icon-download"></i>
            </el-link>
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
      models: { drone: 'drone', ground: 'ground' }, // 模型配置聚合
      streams: { // 视频流配置聚合
        drone: {
          isStreaming: false,
          src: '',
          latestUrl: '',
          detections: [],
          isLoading: false,
          intervals: { refresh: null, detection: null }
        },
        ground: {
          isStreaming: false,
          src: '',
          latestUrl: '',
          detections: [],
          isLoading: false,
          intervals: { refresh: null, detection: null }
        }
      }
    };
  },
  methods: {
    // 模型切换
    async handleModelChange(type) {
      try {
        await this.$request.get(`http://localhost:5000/switch_model?model=${this.models[type]}`);
        this.$message.success(`${type === 'drone' ? '无人机' : '地面端'}模型切换成功`);
      } catch (error) {
        this.$message.error(`${type === 'drone' ? '无人机' : '地面端'}模型切换失败`);
      }
    },

    // 跳转GPS页面
    navigateToGPS() {
      this.$router.push('/GPS');
    },

    // 切换视频流状态
    toggleStream(type) {
      this.streams[type].isStreaming ? this.stopStream(type) : this.startStream(type);
    },

    // 启动视频流
    startStream(type) {
      const stream = this.streams[type];
      stream.isStreaming = true;
      stream.src = `http://localhost:5000/video_feed?type=${type}&t=${Date.now()}`;
      
      // 清除旧定时器
      Object.values(stream.intervals).forEach(interval => interval && clearInterval(interval));
      
      // 启动检测轮询
      stream.intervals.detection = setInterval(() => this.fetchDetections(type), 1000);
    },

    // 停止视频流
    async stopStream(type) {
      const stream = this.streams[type];
      stream.isStreaming = false;
      stream.src = '';
      stream.detections = [];
      
      // 清除检测轮询
      clearInterval(stream.intervals.detection);
      
      // 获取最新视频并启动刷新轮询
      await this.fetchLatestVideo(type);
      stream.intervals.refresh = setInterval(() => !stream.isStreaming && this.fetchLatestVideo(type), 5000);
    },

    // 获取实时检测结果
    async fetchDetections(type) {
      const stream = this.streams[type];
      stream.isLoading = true;
      try {
        const res = await axios.get(`http://localhost:5000/realtime_detections?type=${type}`);
        stream.detections = res.data.detections.map(d => ({
          ...d,
          confidence: parseFloat(d.confidence.toFixed(3))
        }));
      } catch (error) {
        console.error(`${type === 'drone' ? '无人机' : '地面端'}检测数据获取失败`, error);
      } finally {
        stream.isLoading = false;
      }
    },

    // 获取最新录制视频
    async fetchLatestVideo(type) {
      try {
        const res = await this.$request.get(`http://localhost:5000/latest_video?type=${type}`);
        this.streams[type].latestUrl = res.url;
      } catch (error) {
        console.error(`${type === 'drone' ? '无人机' : '地面端'}视频获取失败`, error);
      }
    },

    // 格式化置信度
    formatConfidence(percentage) {
      return `${percentage.toFixed(1)}%`;
    },

    // 置信度颜色映射
    getConfidenceColor(percentage) {
      return percentage > 90 ? '#67C23A' : percentage > 70 ? '#E6A23C' : '#F56C6C';
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
  padding: 20px;
}

.page-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 25px;
  padding-bottom: 15px;
  border-bottom: 1px solid #f0f2f5;
}

.page-title {
  margin: 0;
  font-size: 20px;
  color: #2c3e50;
}

.page-meta {
  color: #606266;
  font-size: 14px;
}

.ai-chat-card {
  margin-right: 10px;
}

.detection-card {
  padding: 25px;
  border-radius: 6px;
  box-shadow: 0 2px 12px rgba(0, 0, 0, 0.05);
}

.video-module {
  margin-bottom: 35px;
  padding-bottom: 20px;
}

.detection-header {
  display: flex;
  align-items: center;
  margin-bottom: 18px;
}

.video-container {
  border: 1px solid #ebeef5;
  border-radius: 6px;
  padding: 15px;
  background-color: #fafafa;
  margin-bottom: 20px;
  min-height: 320px;
  display: flex;
  flex-direction: column;
  justify-content: center;
  align-items: center;
}

.video-stream {
  max-width: 100%;
  height: auto;
  min-height: 300px;
  object-fit: contain;
  border-radius: 4px;
}

.video-links {
  text-align: center;
  padding: 20px;
}

.video-links h3 {
  margin-bottom: 15px;
  font-size: 16px;
  color: #303133;
}

.realtime-results {
  margin-top: 20px;
}

.realtime-results h3 {
  margin: 0 0 15px 0;
  font-size: 16px;
  color: #303133;
  font-weight: 500;
}

.video-separator {
  height: 1px;
  background-color: #ebeef5;
  margin: 30px 0;
}

.el-table {
  border-radius: 6px;
  border: 1px solid #ebeef5;
}

.coordinates {
  display: flex;
  gap: 10px;
}

.coord-item {
  font-family: monospace;
  color: #2c3e50;
}

.raw-coords {
  font-size: 13px;
  color: #606266;
  font-family: monospace;
}

.empty-status {
  text-align: center;
  padding: 30px 0;
  color: #909399;
}

.empty-status i {
  font-size: 36px;
  margin-bottom: 10px;
  display: block;
}

/* 响应式调整 */
@media (max-width: 768px) {
  .page-header {
    flex-direction: column;
    align-items: flex-start;
    gap: 15px;
    margin-bottom: 20px;
  }
  .video-module {
    margin-bottom: 25px;
  }
  .detection-card {
    padding: 15px;
  }
  .video-container {
    min-height: 250px;
    margin-bottom: 15px;
  }
  .video-separator {
    margin: 20px 0;
  }
}
</style>