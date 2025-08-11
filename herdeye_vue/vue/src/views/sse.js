// 全局存储连接实例
let mainEs = null;
let mapEs = null;

// 后端URL配置
const backendUrl = process.env.VUE_APP_BASEURL || 'http://8.152.103.136:9090';

// SSE连接通用方法
export const createSSE = (url, onMsg, onConn) => {
  // 如果已有连接，先关闭
  if (window.eventSource) {
    window.eventSource.close();
  }
  
  const es = new EventSource(url);
  
  // 存储到window，方便全局访问
  window.eventSource = es;
  
  es.addEventListener('connected', onConn);
  es.onmessage = onMsg;
  
  es.onerror = () => {
    console.error('SSE连接错误，尝试重连...');
    setTimeout(() => createSSE(url, onMsg, onConn), 5000);
  };
  
  return es;
};

// 主数据SSE连接
export const connectMainSSE = (callbacks) => {
  // 如果已有连接，先关闭
  if (mainEs) {
    mainEs.close();
  }
  
  mainEs = createSSE(
    `${backendUrl}/sse/subscribe`,
    (ev) => {
      try {
        const data = JSON.parse(ev.data);
        // 调用回调函数处理数据
        if (callbacks.onMessage) {
          callbacks.onMessage(data);
        }
        if (callbacks.onSuccess) {
          callbacks.onSuccess();
        }
      } catch (e) {
        console.error('解析主SSE失败:', e);
        if (callbacks.onError) {
          callbacks.onError(e);
        }
      }
    },
    () => {
      if (callbacks.onConnect) {
        callbacks.onConnect();
      }
    }
  );
  
  return mainEs;
};

// 地图SSE连接
export const connectMapSSE = (callbacks) => {
  // 如果已有连接，先关闭
  if (mapEs) {
    mapEs.close();
  }
  
  mapEs = createSSE(
    `${backendUrl}/sse/subscribe`,
    (ev) => {
      try {
        const data = JSON.parse(ev.data);
        // 调用回调函数处理数据
        if (callbacks.onMessage) {
          callbacks.onMessage(data);
        }
        if (callbacks.onSuccess) {
          callbacks.onSuccess();
        }
      } catch (e) {
        console.error('解析地图SSE失败:', e);
        if (callbacks.onError) {
          callbacks.onError(e);
        }
      }
    },
    () => {
      if (callbacks.onConnect) {
        callbacks.onConnect();
      }
    }
  );
  
  return mapEs;
};

// 关闭主SSE连接
export const closeMainSSE = () => {
  if (mainEs) {
    mainEs.close();
    mainEs = null;
  }
};

// 关闭地图SSE连接
export const closeMapSSE = () => {
  if (mapEs) {
    mapEs.close();
    mapEs = null;
  }
};

// 关闭所有SSE连接
export const closeAllSSE = () => {
  closeMainSSE();
  closeMapSSE();
  if (window.eventSource) {
    window.eventSource.close();
    window.eventSource = null;
  }
};

// Vue 2插件安装方法 - 修复核心问题
export default {
  install: function(Vue) {
    // 确保Vue实例存在
    if (!Vue) {
      console.error('Vue实例未找到');
      return;
    }
    
    // 挂载到Vue原型，组件中可通过this.$sse访问
    Vue.prototype.$sse = {
      createSSE,
      connectMainSSE,
      connectMapSSE,
      closeMainSSE,
      closeMapSSE,
      closeAllSSE
    };
  }
};
