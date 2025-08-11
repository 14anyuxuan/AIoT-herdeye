import Vue from 'vue'
import App from './App.vue'
import router from './router'
import ElementUI from 'element-ui'
import 'element-ui/lib/theme-chalk/index.css'
import '@/assets/css/global.css'
import '@/assets/css/theme/index.css'
import request from "@/utils/request";
import sseService from './services/sseService'  // 导入服务

// 全局挂载 SSE 服务（关键步骤）
Vue.prototype.$sseService = sseService  // 组件中通过 this.$sseService 访问

// 其他配置
Vue.config.productionTip = false
Vue.prototype.$request = request
Vue.prototype.$baseUrl = process.env.VUE_APP_BASEURL
Vue.use(ElementUI, { size: "small" })

// 初始化 SSE 连接
sseService.initConnections()

// 实例化 Vue
new Vue({
  router,
  render: h => h(App),
  beforeDestroy() {
    sseService.closeConnections()  // 应用销毁时关闭连接
  }
}).$mount('#app')