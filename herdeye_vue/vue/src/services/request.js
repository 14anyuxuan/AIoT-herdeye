import axios from 'axios';

// 创建Axios实例并配置超时时间
const request = axios.create({
  timeout: 15000, // 设置15秒超时
  headers: {
    'Content-Type': 'application/json'
  }
});

// 请求拦截器
request.interceptors.request.use(
  config => {
    // 可以在这里添加认证信息等
    return config;
  },
  error => {
    console.error('请求错误:', error);
    return Promise.reject(error);
  }
);

// 响应拦截器
request.interceptors.response.use(
  response => {
    return response.data;
  },
  error => {
    console.error('响应错误:', error);
    // 统一错误处理
    if (error.code === 'ECONNABORTED') {
      alert('请求超时，请检查网络连接或稍后重试');
    }
    return Promise.reject(error);
  }
);

export default request;