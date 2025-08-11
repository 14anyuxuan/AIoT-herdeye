<template>
  <div class="dashboard">
    <!-- 顶部信息栏 -->
    <header class="top-bar">
      <div class="top-left">
        <h1>养殖场环境监控系统</h1>
        <div class="weather-info">
          <span>{{ weather.date }} {{ weather.weekday }}</span>
          <span>{{ weather.weather }}</span>
          <span>{{ weather.tempMin }}°~{{ weather.tempMax }}°</span>
          <span>{{ weather.windDir }} {{ weather.windForce }}</span>
        </div>
      </div>
      <div class="top-right">
        <div class="conn-status">
          <span class="dot" :class="{ green: mainConn }"></span>
          <span>{{ mainConn ? '连接正常' : '连接断开' }}</span>
        </div>
        <span>{{ currDate }}</span>
      </div>
    </header>

    <!-- 核心内容区 -->
    <main class="main-content container">
      <!-- 环境状态指标 -->
      <section class="status-cards panel">
        <h2>环境跟踪</h2>
        <div class="cards">
          <div class="card" :class="{ warn: tempWarn }">
            <div class="card-title">当前温度</div>
            <div class="val">{{ temp !== null ? temp.toFixed(1) + '°C' : '加载中...' }}</div>
            <div class="card-desc">
              <span>正常范围: {{ tempRange.min }}°C - {{ tempRange.max }}°C</span>
            </div>
          </div>
          <div class="card" :class="{ warn: humWarn }">
            <div class="card-title">当前湿度</div>
            <div class="val">{{ hum !== null ? hum.toFixed(1) + '%' : '加载中...' }}</div>
            <div class="card-desc">
              <span>正常范围: {{ humRange.min }}% - {{ humRange.max }}%</span>
            </div>
          </div>
        </div>
      </section>

      <!-- 趋势图表 -->
      <section class="charts panel">
        <h2>环境趋势</h2>
        <div class="charts-grid">
          <div class="chart-box">
            <div class="chart-title">温度历史数据</div>
            <div id="tempChart" class="chart"></div>
          </div>
          <div class="chart-box">
            <div class="chart-title">湿度历史数据</div>
            <div id="humChart" class="chart"></div>
          </div>
        </div>
      </section>

      <!-- 设备与状态 - 优化布局 -->
      <section class="device-info panel">
        <h2>设备与状态信息</h2>
        <!-- 优化后的设备网格布局 -->
        <div class="device-grid">
          <!-- 左侧：水位和重量按列布局 -->
          <div class="device-left-columns">
            <!-- 水位列 -->
            <div class="device-column">
              <div class="device-card water-card" :class="{ warn: waterAlarm }">
                <div class="device-icon">📊</div>
                <div class="device-name">水位</div>
                <div class="device-status">
                  <span class="status-indicator" :class="{ active: water.status === 1 }"></span>
                  <span class="status-text">{{ water.status === 1 ? '不足' : '正常' }}</span>
                </div>
              </div>
            </div>
            
            <!-- 重量列 -->
            <div class="device-column">
              <div class="device-card feed-card">
                <div class="feed-header">
                  <div class="device-icon">🌾</div>
                  <div class="device-name">重量</div>
                </div>
                <div class="feed-content">
                  <div class="vertical-progress">
                    <div class="progress-track" :style="{ height: barHeight + 'px' }">
                      <div 
                        class="progress-fill" 
                        :class="{ low: weightPercent < 20 }"
                        :style="{ 
                          height: weightPercent + '%',
                          background: gradientBg 
                        }"
                      ></div>
                      <div class="progress-marker" :style="{ bottom: weightPercent + '%' }">
                        <div class="marker-line"></div>
                      </div>
                    </div>
                  </div>
                  
                  <div class="feed-data">
                    <div class="primary-value">{{ weight.toFixed(1) }}g</div>
                    <div class="secondary-value">{{ weightPercent }}%</div>
                    <div v-if="weightPercent < 20" class="warning-alert">
                      <i class="warning-icon">⚠️</i>
                      <span>饲料不足，请补充</span>
                    </div>
                  </div>
                </div>
              </div>
            </div>
          </div>
          
          <!-- 右侧：2行3列共6个设备卡片 -->
          <div class="device-right-grid">
            <!-- 第一行设备 -->
            <div class="device-card" :class="{ active: devices[0].status === 1 }">
              <div class="device-icon">💧</div>
              <div class="device-name">水泵</div>
              <div class="device-status">
                <span class="status-indicator" :class="{ active: devices[0].status === 1 }"></span>
                <span class="status-text">{{ devices[0].status === 1 ? '运行中' : '已关闭' }}</span>
              </div>
            </div>
            
            <div class="device-card" :class="{ active: devices[1].status === 1 }">
              <div class="device-icon">🔊</div>
              <div class="device-name">电机</div>
              <div class="device-status">
                <span class="status-indicator" :class="{ active: devices[1].status === 1 }"></span>
                <span class="status-text">{{ devices[1].status === 1 ? '运行中' : '已关闭' }}</span>
              </div>
            </div>
            
            <div class="device-card" :class="{ active: devices[2].status === 1 }">
              <div class="device-icon">💡</div>
              <div class="device-name">灯光</div>
              <div class="device-status">
                <span class="status-indicator" :class="{ active: devices[2].status === 1 }"></span>
                <span class="status-text">{{ devices[2].status === 1 ? '运行中' : '已关闭' }}</span>
              </div>
            </div>
            
            <!-- 第二行设备 -->
            <div class="device-card" :class="{ active: devices[3].status === 1 }">
              <div class="device-icon">🚪</div>
              <div class="device-name">圈门</div>
              <div class="device-status">
                <span class="status-indicator" :class="{ active: devices[3].status === 1 }"></span>
                <span class="status-text">{{ devices[3].status === 1 ? '运行中' : '已关闭' }}</span>
              </div>
            </div>
            
            <div class="device-card" :class="{ active: devices[4].status === 1 }">
              <div class="device-icon">💨</div>
              <div class="device-name">风扇</div>
              <div class="device-status">
                <span class="status-indicator" :class="{ active: devices[4].status === 1 }"></span>
                <span class="status-text">{{ devices[4].status === 1 ? '运行中' : '已关闭' }}</span>
              </div>
            </div>
            
            <div class="device-card" :class="{ active: devices[5].status === 1 }">
              <div class="device-icon">🔥</div>
              <div class="device-name">加热</div>
              <div class="device-status">
                <span class="status-indicator" :class="{ active: devices[5].status === 1 }"></span>
                <span class="status-text">{{ devices[5].status === 1 ? '运行中' : '已关闭' }}</span>
              </div>
            </div>
          </div>
        </div>
      </section>
      

      <!-- 地图区域 -->
      <section class="map-section panel">
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
      <div>最后更新时间: {{ lastUpdate || '未更新' }}</div>
      <button class="refresh" @click="refresh">
        <i class="fa fa-refresh"></i> 刷新数据
      </button>
    </footer>
  </div>
</template>


<script>
import * as echarts from 'echarts'
import markerIcon from '@/assets/imgs/icons8-圈向上-64.png'

export default {
  name: 'FarmMonitor',
  data() {
    return {
      // 核心数据
      weather: {
        date: `${new Date().getMonth() + 1}月${new Date().getDate()}日`,
        weekday: ['周日', '周一', '周二', '周三', '周四', '周五', '周六'][new Date().getDay()],
        weather: '晴',
        tempMin: 18,
        tempMax: 30,
        windDir: '东风',
        windForce: '3级'
      },
      currDate: new Date().toLocaleDateString(),
      loading: true, // 加载状态
      temp: null, // 初始为null，标识未加载
      hum: null, // 初始为null，标识未加载
      tempHistory: [], // 移除默认测试数据，初始为空
      humHistory: [], // 移除默认测试数据，初始为空
      tempRange: { min: 18, max: 26 },
      humRange: { min: 40, max: 60 },

      // WebSocket配置
      mainWs: null,
      backendUrl: 'ws://8.152.103.136:9090/ws/data',

      // 设备数据
      water: { status: 0, value: 0 },
      weight: 50,
      devices: [
        { name: '水泵', status: 0 },
        { name: '电机', status: 0 },
        { name: '灯光', status: 0 },
        { name: '阀门', status: 0 },
        { name: '加热', status: 0 },
        { name: '风机', status: 0 }
      ],

      // 连接与配置状态
      mainConn: false,
      mapConn: false,
      lastUpdate: null, // 初始未更新
      maxHistory: 10,
      barHeight: 120, // 调整进度条高度，使卡片高度一致
      colors: ['#43a047', '#8bc34a', '#cddc39'],
      weightPercent: 0,
      tempWarn: false,
      humWarn: false,
      waterAlarm: false,
      gradientBg: 'linear-gradient(to top, #43a047, #8bc34a, #cddc39)',

      // 地图相关
      lng1: 1,
      lat1: 1,
      lnglat: null, // 初始为null
      course: 1,
      mapStatus: '地图加载中...',
      map: null,
      marker: null,

      // 图表实例
      tempChart: null,
      humChart: null,
      hasReceivedData: false // 标识是否首次接收数据
    }
  },
  methods: {
    // 图表初始化与渲染
    initCharts() {
      const tempDom = document.getElementById('tempChart')
      const humDom = document.getElementById('humChart')
      if (!tempDom || !humDom) return

      this.tempChart = echarts.init(tempDom)
      this.humChart = echarts.init(humDom)
      
      window.addEventListener('resize', () => {
        this.tempChart?.resize()
        this.humChart?.resize()
      })
      this.renderCharts() // 初始渲染（空数据状态）
    },
    renderCharts() {
      // 处理空数据情况：使用默认范围
      const getDefaultRange = (type) => {
        return type === 'temp' 
          ? { min: this.tempRange.min - 2, max: this.tempRange.max + 2 }
          : { min: this.humRange.min - 5, max: this.humRange.max + 5 }
      }

      // 温度图表配置
      const tempValues = this.tempHistory.map(i => i.value)
      const tempTimes = this.tempHistory.map(i => i.time)
      const tempRange = tempValues.length 
        ? { 
            min: Math.min(...tempValues, this.tempRange.min) - 2,
            max: Math.max(...tempValues, this.tempRange.max) + 2
          }
        : getDefaultRange('temp')

      this.tempChart.setOption({
        tooltip: { trigger: 'axis' },
        grid: { left: '3%', right: '4%', bottom: '3%', containLabel: true },
        xAxis: { 
          type: 'category', 
          data: tempTimes.length ? tempTimes : ['暂无数据'],
          axisLabel: { rotate: 30 } 
        },
        yAxis: {
          type: 'value',
          name: '温度(°C)',
          min: tempRange.min,
          max: tempRange.max,
          markLine: {
            data: [
              { yAxis: this.tempRange.min, name: '最低阈值', lineStyle: { color: 'red' } },
              { yAxis: this.tempRange.max, name: '最高阈值', lineStyle: { color: 'red' } }
            ]
          }
        },
        series: [{
          name: '温度',
          type: 'line',
          data: tempValues.length ? tempValues : [null], // 空数据时显示空
          smooth: true,
          lineStyle: { color: '#e74c3c' },
          itemStyle: { color: '#e74c3c' },
          areaStyle: { color: new echarts.graphic.LinearGradient(0,0,0,1,[
            { offset: 0, color: 'rgba(231,76,60,0.3)' },
            { offset: 1, color: 'rgba(231,76,60,0)' }
          ])}
        }]
      })

      // 湿度图表配置
      const humValues = this.humHistory.map(i => i.value)
      const humTimes = this.humHistory.map(i => i.time)
      const humRange = humValues.length 
        ? { 
            min: Math.min(...humValues, this.humRange.min) - 5,
            max: Math.max(...humValues, this.humRange.max) + 5
          }
        : getDefaultRange('hum')

      this.humChart.setOption({
        tooltip: { trigger: 'axis' },
        grid: { left: '3%', right: '4%', bottom: '3%', containLabel: true },
        xAxis: { 
          type: 'category', 
          data: humTimes.length ? humTimes : ['暂无数据'],
          axisLabel: { rotate: 30 } 
        },
        yAxis: {
          type: 'value',
          name: '湿度(%)',
          min: humRange.min,
          max: humRange.max,
          markLine: {
            data: [
              { yAxis: this.humRange.min, name: '最低阈值', lineStyle: { color: 'blue' } },
              { yAxis: this.humRange.max, name: '最高阈值', lineStyle: { color: 'blue' } }
            ]
          }
        },
        series: [{
          name: '湿度',
          type: 'line',
          data: humValues.length ? humValues : [null], // 空数据时显示空
          smooth: true,
          lineStyle: { color: '#3498db' },
          itemStyle: { color: '#3498db' },
          areaStyle: { color: new echarts.graphic.LinearGradient(0,0,0,1,[
            { offset: 0, color: 'rgba(52,152,219,0.3)' },
            { offset: 1, color: 'rgba(52,152,219,0)' }
          ])}
        }]
      })
    },

    // 历史数据更新（增加首次数据初始化逻辑）
    updateHistory(ref, val, isFirst) {
      const timeStr = new Date().toLocaleTimeString()
      if (isFirst) {
        // 首次接收数据：直接初始化（填充maxHistory条相同数据，避免图表突变）
        ref.splice(0, ref.length) // 清空
        for (let i = 0; i < this.maxHistory; i++) {
          ref.push({ time: timeStr, value: val })
        }
      } else {
        // 非首次：正常追加并截断
        ref.push({ time: timeStr, value: val })
        if (ref.length > this.maxHistory) ref.shift()
      }
    },

    // 地图初始化
    initMap() {
      const loadMap = () => new Promise((res, rej) => {
        if (window.AMap) return res(window.AMap)
        const script = document.createElement('script')
        script.src = 'https://webapi.amap.com/maps?v=1.4.15&key=你的高德Key'
        script.onload = () => res(window.AMap || rej(new Error('地图加载失败')))
        script.onerror = () => rej(new Error('地图脚本加载失败'))
        document.head.appendChild(script)
      })

      loadMap().then(AMap => {
        this.map = new AMap.Map('map', {
          resizeEnable: true,
          zoom: 16,
          center: [116.397470, 39.908823],
          viewMode: '2D'
        })
  
        this.marker = new AMap.Marker({
          position: [116.397470, 39.908823],
          icon: new AMap.Icon({
            size: new AMap.Size(32, 32),
            image: markerIcon,
            imageSize: new AMap.Size(64, 64),
            imageOffset: new AMap.Pixel(-16, -16)
          }),
          anchor: 'center'
        })
        this.map.add(this.marker)
        
        this.map.on('complete', () => this.mapStatus = '地图已就绪')
        this.map.on('error', () => this.mapStatus = '地图渲染错误')
      }).catch(err => {
        this.mapStatus = '地图加载失败'
        console.error(err)
      })
    },

    // WebSocket初始化
    initWebSocket() {
      this.mainWs = new WebSocket(this.backendUrl)
      
      this.mainWs.onopen = () => {
        this.mainConn = true
        this.mapConn = true
      }
      
      // 核心数据提取与展示逻辑
      this.mainWs.onmessage = (event) => {
        this.loading = false
        try {
          const data = JSON.parse(event.data)
          console.log('接收数据:', data)
          
          // 环境数据更新 - 添加类型检查
          const newTemp = typeof data.T === 'number' ? data.T : this.temp
          const newHum = typeof data.H === 'number' ? data.H : this.hum
          
          if (newTemp !== null) this.temp = newTemp
          if (newHum !== null) this.hum = newHum
          
          // 阈值警告判断
          this.tempWarn = this.temp !== null && (this.temp < this.tempRange.min || this.temp > this.tempRange.max)
          this.humWarn = this.hum !== null && (this.hum < this.humRange.min || this.hum > this.humRange.max)
        
          // 水位状态更新
          this.water = {
            status: parseInt(data.water_level) || 0,
            value: parseInt(data.water_level) || 0
          }
          this.waterAlarm = this.water.status === 1  // 当水位状态为1（不足）时触发警告
        
          // 饲料重量更新
          this.weight = Math.min(Math.max((data.weight) || 0, 0), 100)
          this.weightPercent = Math.min(Math.max(this.weight, 0), 100)
          
          // 设备状态更新
          this.devices[0].status = parseInt(data.pump) || 0    // 水泵
          this.devices[1].status = parseInt(data.motor) || 0   // 电机
          this.devices[2].status = parseInt(data.light) || 0   // 灯光
          this.devices[3].status = parseInt(data.door) || 0    // 阀门
          this.devices[4].status = parseInt(data.heat) || 0    // 加热
          this.devices[5].status = parseInt(data.fan) || 0     // 风机

          // 更新历史数据与图表（首次数据特殊处理）
          const isFirst = !this.hasReceivedData
          if (this.temp !== null) {
            this.updateHistory(this.tempHistory, this.temp, isFirst)
          }
          if (this.hum !== null) {
            this.updateHistory(this.humHistory, this.hum, isFirst)
          }
          
          // 标记已接收过数据
          if (isFirst) this.hasReceivedData = true
          
          this.lastUpdate = new Date().toLocaleString()
          this.renderCharts()
          
          // 地图位置更新
          if (data.lng && data.lat) {
            this.lng1 = data.lng
            this.lat1 = data.lat
            this.lnglat = [this.lng1, this.lat1]
            this.marker?.setPosition([data.lng + 0.01, data.lat - 0.001])
            this.map?.panTo([(data.lng) + 0.01, (data.lat) - 0.001], { duration: 300 })
          }
          if (data.cogt) this.course = data.cogt
          this.marker?.setRotation((((data.cogt) % 360) + 360) % 360)
          
        } catch (e) {
          console.error('数据解析失败:', e)
        }
      }
      
      this.mainWs.onerror = () => {
        this.mainConn = false
        this.mapConn = false
      }
      
      this.mainWs.onclose = () => {
        this.mainConn = false
        this.mapConn = false
        setTimeout(() => this.initWebSocket(), 3000)
      }
    },
    
    // 刷新数据
    refresh() {
      if (this.mainWs?.readyState === WebSocket.OPEN) {
        this.mainWs.send(JSON.stringify({ type: 'REFRESH_DATA' }))
        this.lastUpdate = new Date().toLocaleString()
      }
    }
  },
  mounted() {
    this.$nextTick(() => {
      this.initCharts()
      this.initMap()
      this.initWebSocket()
    })
  },
  beforeDestroy() {
    // 清理资源
    this.mainWs?.close()
    this.tempChart?.dispose()
    this.humChart?.dispose()
    this.map?.destroy()
  }
}
</script>
<style scoped>
/* 全局样式优化 */
* { 
  margin: 0; 
  padding: 0; 
  box-sizing: border-box; 
}
.dashboard { 
  font-family: 'Microsoft YaHei', 'Segoe UI', Roboto, sans-serif; 
  display: flex; 
  flex-direction: column; 
  background: #f5f7fa; 
  min-height: 100vh; 
  color: #333;
}

/* 顶部栏优化：响应式堆叠 */
.top-bar { 
  display: flex; 
  justify-content: space-between; 
  align-items: center; 
  padding: 1rem 2rem; 
  background: #2c3e50; 
  color: #fff; 
  box-shadow: 0 2px 10px rgba(0,0,0,0.15); 
  flex-wrap: wrap; 
  gap: 1rem; 
  z-index: 10;
}
/* 顶部左右分区，小屏幕自动堆叠 */
.top-left { flex: 1; min-width: 280px; }
.top-left h1 {
  font-size: 1.5rem;
  margin-bottom: 0.3rem;
  display: flex;
  align-items: center;
  gap: 0.5rem;
}
.top-left h1::before {
  content: '📊';
  font-size: 1.8rem;
}
.top-right { flex: 0 0 auto; display: flex; align-items: center; gap: 1.5rem; }
.weather-info { 
  display: flex; 
  gap: 1.5rem; 
  font-size: 0.9rem; 
  color: #e0e0e0; 
  flex-wrap: wrap;  /* 天气信息过多时自动换行 */
}
.conn-status { display: flex; align-items: center; gap: 0.5rem; }
.dot { 
  display: inline-block; 
  width: 10px; 
  height: 10px; 
  border-radius: 50%; 
  background: #e74c3c;
  box-shadow: 0 0 0 rgba(231, 76, 60, 0.4);
  animation: pulse-off 1.5s infinite;
}
.dot.green { 
  background: #27ae60;
  box-shadow: 0 0 0 rgba(39, 174, 96, 0.4);
  animation: pulse-on 1.5s infinite;
}
.currDate { font-size: 0.9rem; }

/* 主内容区优化：容器化+间距统一 */
.main-content { 
  flex: 1; 
  padding: 1.5rem; 
  display: flex; 
  flex-direction: column; 
  gap: 1.5rem;  /* 统一模块间距 */
  overflow-y: auto; 
}
/* 容器限制最大宽度，避免大屏过宽 */
.container { 
  max-width: 1600px; 
  margin: 0 auto;  /* 居中显示 */
  width: 100%; 
  padding: 0 1rem; 
}
/* 面板样式：统一模块外观 */
.panel { 
  background: #fff; 
  border-radius: 10px; 
  box-shadow: 0 4px 15px rgba(0,0,0,0.05); 
  padding: 1.5rem; 
  transition: all 0.3s ease; 
  position: relative;
  overflow: hidden;
}
.panel::before {
  content: '';
  position: absolute;
  top: 0;
  left: 0;
  width: 4px;
  height: 100%;
  background: #3498db;
}
.panel:hover { 
  box-shadow: 0 8px 25px rgba(0,0,0,0.1);
  transform: translateY(-2px);
}
h2 { 
  font-size: 1.3rem; 
  color: #2c3e50; 
  margin-bottom: 1.2rem; 
  padding-left: 0.8rem;
  font-weight: 600;
  position: relative;
}
h2::before {
  content: '';
  position: absolute;
  left: 0;
  top: 50%;
  transform: translateY(-50%);
  width: 4px;
  height: 16px;
  background: #3498db;
  border-radius: 2px;
}

/* 环境状态卡片优化：紧凑布局 */
.status-cards .cards { 
  display: grid; 
  grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));  /* 自适应列数 */
  gap: 1.5rem; 
}
.status-cards .card { 
  background: #fff; 
  border-radius: 8px; 
  box-shadow: 0 3px 10px rgba(0,0,0,0.08); 
  padding: 1.2rem; 
  display: flex; 
  flex-direction: column; 
  gap: 0.8rem; 
  border: 1px solid #f0f0f0;
  transition: all 0.3s ease;
  position: relative;
  overflow: hidden;
}
.status-cards .card::after {
  content: '';
  position: absolute;
  top: 0;
  left: 0;
  width: 100%;
  height: 3px;
  background: linear-gradient(90deg, #3498db, #2980b9);
  transform: scaleX(0);
  transition: transform 0.3s ease;
}
.status-cards .card:hover::after {
  transform: scaleX(1);
}
.status-cards .card:hover { 
  transform: translateY(-5px);
  box-shadow: 0 8px 20px rgba(0,0,0,0.12);
}
.status-cards .card.warn { 
  border-color: #f8d7da;
  background: #fff8f8;
}
.status-cards .card.warn::after {
  background: linear-gradient(90deg, #e74c3c, #c0392b);
}
.status-cards .card-title {
  font-size: 0.95rem;
  color: #666;
  font-weight: 500;
}
.status-cards .val { 
  font-size: 2.2rem; 
  font-weight: 600; 
  color: #2c3e50; 
  line-height: 1.2;
}
.status-cards .card-desc {
  font-size: 0.85rem;
  color: #888;
  margin-top: auto;
}

/* 图表区域优化：响应式自适应 */
.charts .charts-grid { 
  display: grid; 
  grid-template-columns: 1fr;  /* 默认单列，大屏自动调整 */
  gap: 1.5rem; 
  height: 400px; 
}
/* 中等屏幕以上显示双列 */
@media (min-width: 992px) {
  .charts .charts-grid { grid-template-columns: repeat(2, 1fr); }
}
.chart-box { 
  display: flex; 
  flex-direction: column; 
  gap: 0.8rem; 
  background: rgba(255,255,255,0.5);
  padding: 1rem;
  border-radius: 8px;
  box-shadow: 0 2px 8px rgba(0,0,0,0.05);
}
.chart-title {  /* 图表标题样式 */
  font-size: 1rem;
  color: #555;
  padding-left: 0.5rem;
  font-weight: 500;
  display: flex;
  align-items: center;
  gap: 0.5rem;
}
.chart-title::before {
  content: '';
  width: 3px;
  height: 14px;
  background: #3498db;
  border-radius: 2px;
}
.chart-box .chart { 
  width: 100%; 
  height: 100%; 
  min-height: 250px; 
}


/* 设备信息区域优化：列布局样式 */
.device-info { 
  padding-bottom: 1.5rem; 
}

/* 主设备网格容器 */
.device-grid {
  display: flex;
  gap: 1.5rem;
  width: 100%;
  height: 100%;
  min-height: 350px;
}

/* 左侧：水位和重量按列布局容器 */
.device-left-columns {
  flex: 0 0 50%;
  display: grid;
  grid-template-columns: 1fr 1fr; /* 两列布局 */
  gap: 1.5rem;
  height: 100%;
}

/* 每列容器 - 确保高度一致 */
.device-column {
  display: flex;
  flex-direction: column;
  height: 100%;
}

/* 右侧网格：2行3列布局保持不变 */
.device-right-grid {
  flex: 1;
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  grid-template-rows: 1fr 1fr;
  gap: 1.5rem;
  height: 100%;
}

/* 设备卡片样式 - 统一高度设置 */
.device-card {
  display: flex;
  flex-direction: column;
  justify-content: center;
  align-items: center;
  text-align: center;
  padding: 1.2rem;
  transition: all 0.3s ease;
  border-radius: 10px;
  background: #fff;
  box-shadow: 0 4px 12px rgba(0,0,0,0.06);
  border: 1px solid #f0f0f0;
  position: relative;
  overflow: hidden;
  height: 100%; /* 确保卡片占满列高度 */
  min-height: 140px; /* 固定最小高度，确保对齐 */
}

/* 水位卡片特殊设置 - 确保与其他卡片高度一致 */
.water-card {
  min-height: 340px;
  justify-content: center;
  display: flex;
  flex-direction: column;
  align-items: center;
  padding: 1.2rem;
}

/* 专门水位图标位置微调 */
.water-card .device-icon {
  margin-bottom: 1rem; /* 增加图标与下方文字的间距 */
  font-size: 2.4rem; /* 可根据需要调整图标大小 */
  transform: translateY(-5px); /* 向上微调位置 */
}

/* 水位卡片名称和状态位置调整 */
.water-card .device-name {
  margin-bottom: 0.6rem;
}

.water-card .device-status {
  margin-top: 0;
}

/* 饲料卡片样式 - 与其他卡片对齐 */
.feed-card {
  flex: 1;
  flex-direction: column;
  justify-content: center; /* 改为居中对齐 */
  align-items: flex-start;
  text-align: left;
  padding: 1.2rem;
  min-height: 200px; 
}

/* 饲料卡片内部样式调整 - 紧凑布局 */
.feed-header {
  display: flex;
  align-items: center;
  width: 100%;
  margin-bottom: 0.8rem; /* 减少底部间距 */
}
.feed-header .device-icon {
  margin-bottom: 0;
  margin-right: 0.8rem;
}
.feed-content {
  display: flex;
  align-items: center;
  gap: 1.5rem; /* 减少间距 */
  width: 100%;
}

/* 设备卡片内部元素样式 - 优化对齐 */
.device-card::before {
  content: '';
  position: absolute;
  top: 0;
  left: 0;
  width: 100%;
  height: 3px;
  background: transparent;
  transition: background 0.3s ease;
}
.device-card:hover {
  transform: translateY(-5px);
  box-shadow: 0 10px 25px rgba(0,0,0,0.1);
  border-color: #e0e0e0;
}
.device-card:hover::before {
  background: linear-gradient(90deg, #3498db, #2980b9);
}
.device-card.active {
  border-color: #e3f2fd;
  background: #f8fcff;
}
.device-card.active::before {
  background: linear-gradient(90deg, #4caf50, #388e3c);
}
.device-card.warn {
  border-color: #ffebee;
  background: #fff8f8;
}
.device-card.warn::before {
  background: linear-gradient(90deg, #e74c3c, #c62828);
}

.device-icon {
  font-size: 2.2rem;
  margin-bottom: 0.8rem;
  color: #555;
  transition: transform 0.3s ease, color 0.3s ease;
  min-height: 36px; /* 固定图标高度，确保对齐 */
  display: flex;
  align-items: center;
  justify-content: center;
}
.device-name {
  font-weight: 500;
  font-size: 1rem;
  color: #333;
  margin-bottom: 0.8rem;
  min-height: 24px; /* 固定名称高度，确保对齐 */
  display: flex;
  align-items: center;
  justify-content: center;
}
.device-status {
  display: flex;
  align-items: center;
  gap: 0.5rem;
  margin-top: auto; /* 推到卡片底部，确保对齐 */
}
.status-indicator {
  display: inline-block;
  width: 10px;
  height: 10px;
  border-radius: 50%;
  background: #bdbdbd;
  box-shadow: 0 0 0 rgba(189, 189, 189, 0.4);
  transition: all 0.3s ease;
}
.status-indicator.active {
  background: #4caf50;
  box-shadow: 0 0 0 rgba(76, 175, 80, 0.4);
  animation: pulse-on 1.5s infinite;
}
.device-card.warn .status-indicator {
  background: #e74c3c;
  box-shadow: 0 0 0 rgba(231, 76, 60, 0.4);
  animation: pulse-off 1.5s infinite;
}
.status-text {
  font-size: 0.9rem;
  color: #666;
  font-weight: 500;
}
.device-card.active .status-text {
  color: #4caf50;
}
.device-card.warn .status-text {
  color: #e74c3c;
}

/* 进度条样式调整 */
.vertical-progress {
  display: flex;
  flex-direction: column;
  align-items: center;
}
.progress-track {
  width: 40px; /* 适当加宽进度条 */
  background-color: #f0f2f5;
  border-radius: 6px; /* 圆角矩形的圆角半径 */
  position: relative;
  box-shadow: inset 0 2px 4px rgba(0,0,0,0.1);
}
.progress-fill {
  width: 100%;
  position: absolute;
  bottom: 0;
  border-radius: 6px; /* 与轨道保持一致的圆角 */
  transition: height 1s cubic-bezier(0.34, 1.56, 0.64, 1);
  box-shadow: 0 2px 4px rgba(0,0,0,0.15);
}
/* 调整标记点位置以适应圆角矩形 */
.progress-marker {
  position: absolute;
  width: 34px; /* 适配加宽的进度条 */
  left: 50%;
  transform: translateX(-50%);
  display: flex;
  justify-content: center;
  z-index: 2;
}
.marker-line {
  width: 100%; /* 横线宽度占满容器 */
  height: 4px; /* 横线厚度 */
  background-color: #333; /* 黑色横线 */
  border-radius: 2px; /* 轻微圆角使边缘更柔和 */
  box-shadow: 0 1px 3px rgba(0,0,0,0.2);
  transform: translateY(50%); /* 垂直居中对齐 */
}
.progress-labels {
  display: flex;
  justify-content: space-between;
  width: 40px;
  margin-top: 0.5rem;
  font-size: 0.75rem;
  color: #888;
}
.feed-data {
  display: flex;
  flex-direction: column;
  gap: 0.4rem; /* 减少间距 */
}
.primary-value {
  font-size: 1.8rem; /* 稍微减小字体 */
  font-weight: 600;
  color: #2c3e50;
}
.secondary-value {
  font-size: 1rem;
  color: #666;
  display: flex;
  align-items: center;
  gap: 0.3rem;
}
.secondary-value::after {
  content: '';
  width: 30px;
  height: 1px;
  background-color: #eee;
}
.warning-alert {
  display: flex;
  align-items: center;
  gap: 0.5rem;
  color: #e74c3c;
  font-size: 0.85rem; /* 减小字体 */
  font-weight: 500;
  padding: 0.3rem 0.5rem; /* 减少内边距 */
  background-color: #fff8f8;
  border-radius: 4px;
  margin-top: 0.2rem;
  animation: shake 1.5s infinite;
}
.warning-icon {
  font-size: 0.9rem;
}

/* 地图区域优化：高度自适应 */
.map-section { 
  padding-bottom: 1.5rem; 
}
.map-container { 
  width: 100%; 
  height: 500px;  /* 降低默认高度 */
  position: relative; 
  border-radius: 8px; 
  overflow: hidden; 
  border: 1px solid #f0f0f0;
  box-shadow: 0 2px 10px rgba(0,0,0,0.05);
}
/* 小屏幕地图高度降低 */
@media (max-width: 768px) {
  .map-container { height: 350px; }
}
.map-view { width: 100%; height: 100%; }
.map-status { 
  position: absolute; 
  top: 15px; 
  right: 15px; 
  background: rgba(255,255,255,0.95);
  padding: 0.6rem 1rem; 
  border-radius: 6px; 
  box-shadow: 0 3px 10px rgba(0,0,0,0.1); 
  font-size: 0.9rem; 
  z-index: 10;  /* 确保在地图之上 */
  transition: all 0.3s ease;
}
.map-status:hover {
  box-shadow: 0 5px 15px rgba(0,0,0,0.15);
}
.map-dot { display: inline-block; width: 8px; height: 8px; border-radius: 50%; background: #e74c3c; margin-left: 5px; }
.map-dot.green { background: #27ae60; }
.map-data { 
  position: absolute; 
  bottom: 15px; 
  left: 15px; 
  background: rgba(255,255,255,0.95); 
  padding: 0.8rem; 
  border-radius: 6px; 
  box-shadow: 0 3px 10px rgba(0,0,0,0.1); 
  font-size: 0.9rem; 
  display: flex; 
  flex-wrap: wrap;  /* 小屏幕自动换行 */
  gap: 1.2rem; 
  z-index: 10;
  transition: all 0.3s ease;
}
.map-data:hover {
  box-shadow: 0 5px 15px rgba(0,0,0,0.15);
}
.map-data > div { display: flex; gap: 0.5rem; }
.map-data > div > span:first-child {
  font-weight: 500;
  color: #555;
}


/* 底部区域优化 */
.footer { 
  height: 60px;  /* 增加高度，提升点击区域 */
  display: flex; 
  justify-content: space-between; 
  align-items: center; 
  padding: 0 2rem; 
  background: #2c3e50; 
  color: #fff; 
  font-size: 0.9rem; 
  box-shadow: 0 -2px 10px rgba(0,0,0,0.1);
}
.refresh { 
  background: #3498db; 
  color: #fff; 
  border: none; 
  border-radius: 6px; 
  padding: 0.5rem 1.2rem;  /* 扩大点击区域 */
  display: flex; 
  align-items: center; 
  gap: 0.5rem; 
  cursor: pointer; 
  transition: all 0.3s ease;
  font-weight: 500;
  box-shadow: 0 2px 5px rgba(0,0,0,0.1);
}
.refresh:hover { 
  background: #2980b9; 
  transform: translateY(-2px);
  box-shadow: 0 4px 8px rgba(0,0,0,0.15);
}
.refresh:active { 
  transform: translateY(0);
  box-shadow: 0 2px 3px rgba(0,0,0,0.1);
}


/* 响应式断点细化 */
/* 超小屏幕（手机） */
@media (max-width: 576px) {
  .top-bar { padding: 0.8rem 1rem; }
  .weather-info { gap: 0.8rem; font-size: 0.85rem; }
  .main-content { padding: 1rem 0.5rem; gap: 1rem; }
  .panel { padding: 1rem; }
  .device-grid { flex-direction: column; }
  .device-left-columns { flex: 1; }
  .device-right-grid { 
    grid-template-columns: repeat(2, 1fr);
    grid-template-rows: repeat(3, 1fr);
  }
  .map-data { gap: 0.8rem; padding: 0.6rem; font-size: 0.85rem; }
  .status-cards .val { font-size: 1.8rem; }
  .feed-content { gap: 1rem; }
  .primary-value { font-size: 1.6rem; }
}

/* 小屏幕（平板竖屏） */
@media (min-width: 577px) and (max-width: 768px) {
  .device-grid { flex-direction: column; }
  .device-left-columns, .device-right-grid { flex: 1; }
  .device-right-grid { grid-template-columns: repeat(3, 1fr); }
  .charts .charts-grid { height: 350px; }
}

/* 中等屏幕（平板横屏） */
@media (min-width: 769px) and (max-width: 992px) {
  .device-left-columns { flex: 0 0 35%; }
}

/* 大屏幕（桌面） */
@media (min-width: 993px) and (max-width: 1200px) {
  .device-left-columns { flex: 0 0 28%; }
}

/* 超大屏幕 */
@media (min-width: 1201px) {
  .device-left-columns { flex: 0 0 25%; }
}

/* 动画效果 */
@keyframes pulse { 
  0% { 
    opacity: 1;
    box-shadow: 0 0 0 0 rgba(231, 76, 60, 0.4);
  } 
  70% { 
    opacity: 0.7;
    box-shadow: 0 0 0 8px rgba(231, 76, 60, 0);
  } 
  100% { 
    opacity: 1;
    box-shadow: 0 0 0 0 rgba(231, 76, 60, 0);
  }
}

@keyframes pulse-on {
  0% {
    box-shadow: 0 0 0 0 rgba(76, 175, 80, 0.4);
  }
  70% {
    box-shadow: 0 0 0 8px rgba(76, 175, 80, 0);
  }
  100% {
    box-shadow: 0 0 0 0 rgba(76, 175, 80, 0);
  }
}

@keyframes pulse-off {
  0% {
    box-shadow: 0 0 0 0 rgba(231, 76, 60, 0.4);
  }
  70% {
    box-shadow: 0 0 0 8px rgba(231, 76, 60, 0);
  }
  100% {
    box-shadow: 0 0 0 0 rgba(231, 76, 60, 0);
  }
}

@keyframes shake {
  0%, 100% { transform: translateX(0); }
  25% { transform: translateX(-3px); }
  75% { transform: translateX(3px); }
}

/* 加载状态动画 */
@keyframes fadeIn {
  from { opacity: 0; transform: translateY(10px); }
  to { opacity: 1; transform: translateY(0); }
}

.panel, .device-card, .status-cards .card {
  animation: fadeIn 0.5s ease forwards;
}

.panel:nth-child(1) { animation-delay: 0.1s; }
.panel:nth-child(2) { animation-delay: 0.2s; }
.panel:nth-child(3) { animation-delay: 0.3s; }
.panel:nth-child(4) { animation-delay: 0.4s; }
</style>
