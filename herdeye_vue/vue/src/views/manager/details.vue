<template>
  <div class="dashboard">
    <!-- 顶部信息栏 -->
    <header class="top-bar">
      <div class="title-section">
        <h1>养殖场环境监控系统</h1>
        <div class="weather-info">
          <span>{{ weather.date }} {{ weather.weekday }}</span>
          <span>{{ weather.weather }}</span>
          <span>{{ weather.tempMin }}°~{{ weather.tempMax }}°</span>
          <span>{{ weather.windDir }} {{ weather.windForce }}</span>
        </div>
      </div>
      <div class="status-info">
        <div class="conn-status">
          <span class="dot" :class="{ green: mainConn }"></span>
          <span>{{ mainConn ? '连接正常' : '连接断开' }}</span>
        </div>
        <span>{{ currDate }}</span>
      </div>
    </header>

    <!-- 核心内容区 -->
    <main class="main-content">
      <!-- 环境状态指标 -->
      <section class="status-cards">
        <h2>环境跟踪</h2>
        <div class="cards">
          <div class="card" :class="{ warn: tempWarn }">
            <div>当前温度</div>
            <div class="val">{{ temp.toFixed(1) }}°C</div>
          </div>
          <div class="card" :class="{ warn: humWarn }">
            <div>当前湿度</div>
            <div class="val">{{ hum.toFixed(1) }}%</div>
          </div>
        </div>
      </section>

      <!-- 趋势图表 -->
      <section class="charts">
        <h2>环境趋势</h2>
        <div class="charts-grid">
          <div class="chart-box">
            <div>温度历史数据</div>
            <div id="tempChart" class="chart"></div>
          </div>
          <div class="chart-box">
            <div>湿度历史数据</div>
            <div id="humChart" class="chart"></div>
          </div>
        </div>
      </section>

      <!-- 设备与状态 -->
      <section class="device-info">
        <h2>设备与状态信息</h2>
        <div class="info-grid">
          <div class="status-group">
            <div class="flex-row">
              <!-- 水位状态 -->
              <div class="card" :class="{ warn: waterAlarm }">
                <div>水位状态</div>
                <span class="status" :class="{ on: water.status === 1 }">
                  {{ water.status === 1 ? '正常' : '有误' }}
                </span>
              </div>
              
              <!-- 饲料重量 - 整合后的进度条 -->
              <div class="card feed-card">
                <div>饲料重量状态值</div>
                <div class="progress-container">
                  <!-- 垂直进度条主体 -->
                  <div class="progress-bar" :style="{ height: barHeight + 'px' }">
                    <!-- 进度填充部分 -->
                    <div 
                      class="progress-fill" 
                      :class="{ low: weightPercent < 20 }"
                      :style="{ 
                        height: weightPercent + '%',
                        background: gradientBg 
                      }"
                    ></div>
                    
                    <!-- 进度标记线 -->
                    <div 
                      class="progress-marker"
                      :style="{ bottom: weightPercent + '%' }"
                    ></div>
                  </div>
                  
                  <!-- 重量信息显示 -->
                  <div class="weight-info">
                    <span>{{ weight.toFixed(1) }}g</span>
                    <span>{{ weightPercent }}%</span>
                    <!-- 低电量提示 -->
                    <span v-if="weightPercent < 20" class="low-warning">
                      饲料不足，请补充!
                    </span>
                  </div>
                </div>
              </div>
            </div>
          </div>

          <!-- 设备列表 -->
          <div class="device-list">
            <div v-for="(dev, index) in devices" :key="index" class="device-item">
              <span class="icon">
                <i v-if="dev.name === '水泵'">💧</i>
                <i v-if="dev.name === '电机'">🔊</i>
                <i v-if="dev.name === '灯光'">💡</i>
                <i v-if="dev.name === '阀门'">🚪</i>
              </span>
              <span>{{ dev.name }}:</span>
              <span class="status" :class="{ on: dev.status === 1 }">
                {{ dev.status === 1 ? '运行中' : '已关闭' }}
              </span>
            </div>
          </div>
        </div>
      </section>

      <!-- 地图区域 -->
      <section class="map-section">
        <h2>位置监控</h2>
        <div class="map-container">
          <div id="map" class="map-view"></div>
          <div class="map-status">
            <span>地图后端连接: </span>
            <span :class="{ online: mapConn }">{{ mapConn ? '已连接' : '已断开' }}</span>
            <span class="map-dot" :class="{ green: mapConn }"></span>
          </div>
          <div class="map-data">
            <div><span>经纬度:</span> <span>{{ lnglat || '等待数据...' }}</span></div>
            <div><span>航向角:</span> <span>{{ course || '0°' }}</span></div>
            <div><span>地图状态:</span> <span>{{ mapStatus }}</span></div>
          </div>
        </div>
      </section>
    </main>

    <footer class="footer">
      <div>最后更新时间: {{ lastUpdate }}</div>
      <button class="refresh" @click="refresh">
        <i class="fa fa-refresh"></i> 刷新数据
      </button>
    </footer>
  </div>
</template>

<script setup>
import { ref, onMounted, onUnmounted, computed, nextTick } from 'vue'
import * as echarts from 'echarts'
import markerIcon from '@/assets/imgs/icons8-圈向上-64.png'

// 核心数据
const weather = ref({
  date: `${new Date().getMonth() + 1}月${new Date().getDate()}日`,
  weekday: ['周日', '周一', '周二', '周三', '周四', '周五', '周六'][new Date().getDay()],
  weather: '晴',
  tempMin: 18,
  tempMax: 30,
  windDir: '东风',
  windForce: '3级'
})
const currDate = ref(new Date().toLocaleDateString())
const temp = ref(22)
const hum = ref(50)
const tempHistory = ref([{ time: '00:00', value: 22 }, { time: '00:05', value: 22.5 }])
const humHistory = ref([{ time: '00:00', value: 50 }, { time: '00:05', value: 49.5 }])
const tempRange = { min: 18, max: 26 }
const humRange = { min: 40, max: 60 }

// 设备数据
const water = ref({ status: 0, value: 0 })
const weight = ref(50)
const devices = ref([
  { name: '水泵', status: 0 },
  { name: '电机', status: 0 },
  { name: '灯光', status: 0 },
  { name: '阀门', status: 0 }
])

// 连接状态
const mainConn = ref(false)
const lastUpdate = ref('-')
const backendUrl = 'http://8.152.103.136:9090'
const maxHistory = 10
let mainEs, mapEs

// 进度条相关配置
const barHeight = ref(120) // 进度条高度
const colors = ref(['#43a047', '#8bc34a', '#cddc39']) // 进度条渐变颜色

// 计算属性
const weightPercent = computed(() => Math.max(0, Math.min(100, Math.round(weight.value))))
const tempWarn = computed(() => temp.value < tempRange.min || temp.value > tempRange.max)
const humWarn = computed(() => hum.value < humRange.min || hum.value > humRange.max)
const waterAlarm = computed(() => water.value.value > 8.5 || water.value.value < 2.0)
// 生成渐变背景
const gradientBg = computed(() => `linear-gradient(to top, ${colors.value.join(', ')})`)

// 地图相关
const lnglat = ref('')
const course = ref('')
const mapStatus = ref('地图加载中...')
const mapConn = ref(false)
let map, marker

// 图表实例
let tempChart, humChart

// 图表初始化与渲染
const initCharts = () => {
  const tempDom = document.getElementById('tempChart')
  const humDom = document.getElementById('humChart')
  if (!tempDom || !humDom) return

  tempChart = echarts.init(tempDom)
  humChart = echarts.init(humDom)
  
  window.addEventListener('resize', () => {
    tempChart?.resize()
    humChart?.resize()
  })
  renderCharts()
}

const renderCharts = () => {
  // 温度图表
  tempChart.setOption({
    tooltip: { trigger: 'axis' },
    grid: { left: '3%', right: '4%', bottom: '3%', containLabel: true },
    xAxis: { 
      type: 'category', 
      data: tempHistory.value.map(i => i.time),
      axisLabel: { rotate: 30 } 
    },
    yAxis: {
      type: 'value',
      name: '温度(°C)',
      min: Math.min(...tempHistory.value.map(i => i.value), tempRange.min) - 2,
      max: Math.max(...tempHistory.value.map(i => i.value), tempRange.max) + 2,
      markLine: {
        data: [
          { yAxis: tempRange.min, name: '最低阈值', lineStyle: { color: 'red' } },
          { yAxis: tempRange.max, name: '最高阈值', lineStyle: { color: 'red' } }
        ]
      }
    },
    series: [{
      name: '温度',
      type: 'line',
      data: tempHistory.value.map(i => i.value),
      smooth: true,
      lineStyle: { color: '#e74c3c' },
      itemStyle: { color: '#e74c3c' },
      areaStyle: { color: new echarts.graphic.LinearGradient(0,0,0,1,[
        { offset: 0, color: 'rgba(231,76,60,0.3)' },
        { offset: 1, color: 'rgba(231,76,60,0)' }
      ])}
    }]
  })

  // 湿度图表
  humChart.setOption({
    tooltip: { trigger: 'axis' },
    grid: { left: '3%', right: '4%', bottom: '3%', containLabel: true },
    xAxis: { type: 'category', data: humHistory.value.map(i => i.time), axisLabel: { rotate: 30 } },
    yAxis: {
      type: 'value',
      name: '湿度(%)',
      min: Math.min(...humHistory.value.map(i => i.value), humRange.min) - 5,
      max: Math.max(...humHistory.value.map(i => i.value), humRange.max) + 5,
      markLine: {
        data: [
          { yAxis: humRange.min, name: '最低阈值', lineStyle: { color: 'blue' } },
          { yAxis: humRange.max, name: '最高阈值', lineStyle: { color: 'blue' } }
        ]
      }
    },
    series: [{
      name: '湿度',
      type: 'line',
      data: humHistory.value.map(i => i.value),
      smooth: true,
      lineStyle: { color: '#3498db' },
      itemStyle: { color: '#3498db' },
      areaStyle: { color: new echarts.graphic.LinearGradient(0,0,0,1,[
        { offset: 0, color: 'rgba(52,152,219,0.3)' },
        { offset: 1, color: 'rgba(52,152,219,0)' }
      ])}
    }]
  })
}

// SSE连接通用方法
const createSSE = (url, onMsg, onConn) => {
  const es = new EventSource(url)
  es.addEventListener('connected', onConn)
  es.onmessage = onMsg
  es.onerror = () => setTimeout(() => createSSE(url, onMsg, onConn), 5000)
  return es
}

// 主数据SSE连接
const connectMainSSE = () => {
  mainEs = createSSE(`${backendUrl}/sse/subscribe`, (ev) => {
    try {
      const data = JSON.parse(ev.data)
      temp.value = data.tem || data.T || temp.value
      hum.value = data.hum || data.H || hum.value
      
      water.value = {
        status: data.waterStatus || 0,
        value: data.Wal || data.waterlevel || 0
      }
      
      weight.value = Math.max(0, Math.min(100, parseFloat(data.weight) || 0))

      // 更新设备状态
      devices.value.find(d => d.name === '水泵').status = data.pump || 0
      devices.value.find(d => d.name === '报警电机').status = data.motor || 0
      devices.value.find(d => d.name === '灯光').status = data.light || 0
      devices.value.find(d => d.name === '阀门').status = data.door || 0
      
      // 更新历史数据
      const timeStr = new Date().toLocaleTimeString()
      updateHistory(tempHistory, temp.value, timeStr)
      updateHistory(humHistory, hum.value, timeStr)
      
      lastUpdate.value = new Date().toLocaleString()
      currDate.value = new Date().toLocaleDateString()
      renderCharts()
      mainConn.value = true
    } catch (e) {
      mainConn.value = false
      console.error('解析主SSE失败:', e)
    }
  }, () => mainConn.value = true)
}

// 地图SSE连接
const connectMapSSE = () => {
  mapEs = createSSE(`${backendUrl}/sse/subscribe`, (ev) => {
    try {
      const data = JSON.parse(ev.data)
      if (data.lng !== undefined && data.lat !== undefined) {
        const lng = parseFloat(data.lng), lat = parseFloat(data.lat)
        if (!isNaN(lng) && !isNaN(lat) && lng >= -180 && lng <= 180 && lat >= -90 && lat <= 90) {
          lnglat.value = `${lng.toFixed(6)},${lat.toFixed(6)}`
          marker?.setPosition([lng + 0.01, lat - 0.001])
          map?.panTo([lng + 0.01, lat - 0.001], { duration: 300 })
        }
      }
      if (data.cogt !== undefined) {
        const angle = parseFloat(data.cogt)
        if (!isNaN(angle)) {
          course.value = `${angle.toFixed(1)}°`
          marker?.setRotation(((angle % 360) + 360) % 360)
        }
      }
      mapConn.value = true
    } catch (e) {
      mapConn.value = false
      console.error('解析地图SSE失败:', e)
    }
  }, () => mapConn.value = true)
}

// 历史数据更新
const updateHistory = (ref, val, time) => {
  ref.value.push({ time, value: val })
  if (ref.value.length > maxHistory) ref.value.shift()
}

// 地图初始化
const initMap = () => {
  const loadMap = () => new Promise((res, rej) => {
    if (window.AMap) return res(window.AMap)
    const script = document.createElement('script')
    script.src = 'https://webapi.amap.com/maps?v=1.4.15&key=你的高德Key'
    script.onload = () => res(window.AMap || rej(new Error('地图加载失败')))
    script.onerror = () => rej(new Error('地图脚本加载失败'))
    document.head.appendChild(script)
  })

  loadMap().then(AMap => {
    map = new AMap.Map('map', {
      resizeEnable: true,
      zoom: 16,
      center: [116.397470, 39.908823],
      viewMode: '2D'
    })
    
    marker = new AMap.Marker({
      position: [116.397470, 39.908823],
      icon: new AMap.Icon({
        size: new AMap.Size(32, 32),
        image: markerIcon,
        imageSize: new AMap.Size(64, 64),
        imageOffset: new AMap.Pixel(-16, -16)
      }),
      anchor: 'center'
    })
    map.add(marker)
    
    map.on('complete', () => mapStatus.value = '地图已就绪')
    map.on('error', () => mapStatus.value = '地图渲染错误')
  }).catch(err => {
    mapStatus.value = '地图加载失败'
    console.error(err)
  })
}

// 数据刷新
const refresh = () => {
  mainEs?.close()
  mapEs?.close()
  connectMainSSE()
  connectMapSSE()
  renderCharts()
  lastUpdate.value = new Date().toLocaleString()
}

// 生命周期
onMounted(() => {
  nextTick(() => {
    initCharts()
    connectMainSSE()
    initMap()
    connectMapSSE()
  })
})

onUnmounted(() => {
  mainEs?.close()
  mapEs?.close()
  tempChart?.dispose()
  humChart?.dispose()
  map?.destroy()
})
</script>

<style scoped>
* { margin: 0; padding: 0; box-sizing: border-box; }
.dashboard { font-family: 'Microsoft YaHei', sans-serif; display: flex; flex-direction: column; background: #f5f7fa; min-height: 100vh; }

/* 顶部栏 */
.top-bar { display: flex; justify-content: space-between; align-items: center; padding: 1rem 2rem; background: #2c3e50; color: #fff; box-shadow: 0 2px 4px rgba(0,0,0,0.1); flex-wrap: wrap; gap: 1rem; }
.weather-info { display: flex; gap: 1.5rem; font-size: 0.9rem; color: #e0e0e0; }
.conn-status { display: flex; align-items: center; gap: 0.5rem; }
.dot { display: inline-block; width: 10px; height: 10px; border-radius: 50%; background: #e74c3c; }
.dot.green { background: #27ae60; }
.currDate { font-size: 0.9rem; }

/* 主内容区 */
.main-content { flex: 1; padding: 1.5rem 2rem; display: flex; flex-direction: column; gap: 2rem; overflow-y: auto; }
h2 { font-size: 1.2rem; color: #2c3e50; margin-bottom: 1rem; padding-bottom: 0.5rem; border-bottom: 1px solid #e0e0e0; }

/* 状态卡片 */
.status-cards .cards { display: grid; grid-template-columns: repeat(2, 1fr); gap: 1rem; height: 140px; }
.card { background: #fff; border-radius: 6px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); padding: 1rem; display: flex; flex-direction: column; justify-content: center; gap: 0.5rem; }
.card.warn { border-left: 4px solid #e74c3c; }
.card .val { font-size: 1.8rem; font-weight: bold; color: #2c3e50; }

/* 图表区域 */
.charts .charts-grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 1.5rem; height: 400px; }
.chart-box .chart { width: 100%; height: 100%; min-height: 250px; }

/* 设备信息区 */
.device-info .info-grid { display: flex; gap: 1.5rem; height: 180px; }
.status-group, .device-list { flex: 1; background: #fff; border-radius: 6px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); padding: 1rem; height: 100%; }
.flex-row { display: flex; width: 100%; gap: 1rem; height: 100%; }

/* 整合后的饲料重量进度条样式 */
.feed-card .progress-container { display: flex; align-items: center; gap: 1rem; justify-content: center; flex: 1; }

/* 进度条主体 */
.progress-bar {
  width: 60px;
  border: 2px solid #e0e0e0;
  border-radius: 10px;
  position: relative;
  background-color: #f5f5f5;
  box-shadow: 0 0 10px rgba(0, 0, 0, 0.1) inset;
}

/* 进度填充样式 */
.progress-fill {
  width: 100%;
  position: absolute;
  bottom: 0;
  transition: height 0.8s ease;
  border-radius: 8px;
}

/* 进度标记线 */
.progress-marker {
  position: absolute;
  width: 100%;
  height: 3px;
  background-color: #333;
  transform: translateY(50%);
  box-shadow: 0 0 5px rgba(0, 0, 0, 0.3);
}

/* 低电量样式 */
.progress-fill.low { 
  background: linear-gradient(to top, #e74c3c, #ff5252) !important;
  animation: pulse 1.5s infinite; 
}

/* 重量信息显示 */
.weight-info { 
  display: flex; 
  flex-direction: column; 
  gap: 0.5rem; 
  align-items: center; 
}
.weight-info span {
  font-weight: 500;
}
.low-warning {
  color: #e74c3c;
  font-size: 0.8rem;
  font-weight: bold;
  animation: shake 1s infinite;
}

/* 设备列表 */
.device-list { display: flex; flex-direction: column; gap: 0.8rem; }
.device-item { padding: 0.6rem 0.8rem; border-radius: 4px; background: #fafafa; font-size: 0.9rem; display: flex; align-items: center; gap: 0.8rem; }
.icon { font-size: 1.1rem; }
.status { font-weight: 500; }
.status.on { color: #27ae60; }

/* 地图区域 */
.map-container { width: 100%; height: 600px; position: relative; border-radius: 6px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); overflow: hidden; }
.map-view { width: 100%; height: 100%; }
.map-status { position: absolute; top: 15px; right: 15px; background: rgba(255,255,255,0.9); padding: 0.5rem 1rem; border-radius: 4px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); font-size: 0.85rem; }
.map-dot { display: inline-block; width: 8px; height: 8px; border-radius: 50%; background: #e74c3c; margin-left: 5px; }
.map-dot.green { background: #27ae60; }
.map-data { position: absolute; bottom: 15px; left: 15px; background: rgba(255,255,255,0.9); padding: 0.8rem; border-radius: 6px; box-shadow: 0 2px 8px rgba(0,0,0,0.15); font-size: 0.9rem; display: flex; gap: 1.5rem; }
.map-data > div { display: flex; gap: 0.5rem; }

/* 底部区域 */
.footer { height: 50px; display: flex; justify-content: space-between; align-items: center; padding: 0 2rem; background: #2c3e50; color: #fff; font-size: 0.9rem; }
.refresh { background: #3498db; color: #fff; border: none; border-radius: 4px; padding: 0.4rem 0.8rem; display: flex; align-items: center; gap: 0.5rem; cursor: pointer; }
.refresh:hover { background: #2980b9; }

/* 响应式 */
@media (max-width: 768px) {
  .charts-grid { grid-template-columns: 1fr; height: auto; gap: 1rem; }
  .chart-box { height: 300px; }
  .info-grid { flex-direction: column; height: auto; gap: 1rem; }
  .status-group, .device-list { height: auto; min-height: 150px; }
  .map-container { height: 400px; }
}

/* 动画效果 */
@keyframes pulse { 
  0% { opacity: 1; } 
  50% { opacity: 0.6; } 
  100% { opacity: 1; } 
}

@keyframes shake {
  0%, 100% { transform: translateX(0); }
  25% { transform: translateX(-3px); }
  75% { transform: translateX(3px); }
}
</style>