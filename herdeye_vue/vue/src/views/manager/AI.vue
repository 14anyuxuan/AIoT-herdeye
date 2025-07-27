<template>
  <div class="page-container">
    <!-- 外部页面展示区域 -->
    <div class="iframe-container">
      <!-- 状态展示 -->
      <div v-if="loadingState !== 'loaded'" class="status-overlay">
        <div v-if="loadingState === 'loading'" class="loading-indicator">
          <div class="spinner"></div>
          <p>正在加载页面...</p>
        </div>
        <div v-else class="error-message">
          <i class="error-icon">⚠️</i>
          <p>无法加载页面，请检查网络连接或稍后再试</p>
          <button @click="reloadPage" class="reload-button">重新加载</button>
        </div>
      </div>
      
      <!-- iframe嵌入外部页面 -->
      <div class="iframe-wrapper" v-if="loadingState === 'loaded'">
        <div class="top-bar-mask">
          <span class="mask-title">牧知，随叫随到的牧场专家</span>
        </div>
        <iframe
          :src="targetUrl"
          frameborder="0"
          class="embedded-page"
          @load="onIframeLoaded"
          @error="onIframeError"
        ></iframe>
      </div>
    </div>
  </div>
</template>

<script>
export default {
  name: 'ExternalPageViewer',
  data() {
    return {
      targetUrl: 'http://8.152.103.136:3000',
      loadingState: 'loading' // loading, error, loaded
    }
  },
  methods: {
    onIframeLoaded() {
      this.loadingState = 'loaded';
    },
    onIframeError() {
      this.loadingState = 'error';
    },
    reloadPage() {
      this.loadingState = 'loading';
      this.targetUrl = '';
      this.$nextTick(() => {
        this.targetUrl = 'http://8.152.103.136:3000';
      });
    }
  }
}
</script>

<style scoped>
.page-container {
  width: 100vw;
  height: 100vh;
  overflow: hidden;
  background-color: #f5f5f5;
}

.iframe-container {
  position: relative;
  width: 100%;
  height: 100%;
}

.status-overlay {
  position: absolute;
  inset: 0;
  display: flex;
  justify-content: center;
  align-items: center;
  background-color: #fff;
  z-index: 5;
}

.loading-indicator, .error-message {
  text-align: center;
  padding: 24px;
}

.spinner {
  width: 40px;
  height: 40px;
  margin: 0 auto 16px;
  border: 4px solid #f3f3f3;
  border-top: 4px solid #42b983;
  border-radius: 50%;
  animation: spin 1s linear infinite;
}

@keyframes spin {
  0% { transform: rotate(0deg); }
  100% { transform: rotate(360deg); }
}

.error-icon {
  display: block;
  font-size: 48px;
  margin-bottom: 16px;
  color: #ff4d4f;
}

.error-message p {
  margin: 0 0 24px;
  font-size: 16px;
  color: #333;
  max-width: 500px;
}

.reload-button {
  padding: 8px 16px;
  background-color: #42b983;
  color: white;
  border: none;
  border-radius: 4px;
  cursor: pointer;
  font-size: 14px;
  transition: background-color 0.3s;
}

.reload-button:hover {
  background-color: #359e75;
}

.iframe-wrapper {
  position: relative;
  width: 100%;
  height: 100%;
}

.top-bar-mask {
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  height: 60px;
  background-color: #fff;
  z-index: 4;
  box-shadow: 0 2px 4px rgba(0,0,0,0.1);
  display: flex;
  align-items: center;
  padding: 0 20px;
}

.mask-title {
  font-size: 18px;
  font-weight: 600;
  color: #2c3e50;
  letter-spacing: 0.5px;
}

.embedded-page {
  width: 100%;
  height: 100%;
  border: none;
}
</style>