<template>
  <div class="container">
    <!-- 搜索区域 -->
    <div class="search">
      <el-input placeholder="请输入标题查询" style="width: 200px" v-model="title"></el-input>
      <el-button type="info" plain style="margin-left: 10px" @click="load(1)">查询</el-button>
      <el-button type="warning" plain style="margin-left: 10px" @click="reset">重置</el-button>
      <el-button type="danger" plain @click="delBatch">批量删除</el-button>
    </div>

    <!-- 表格区域 -->
    <div class="table">
      <el-table
          :data="tableData"
          stripe
          @selection-change="handleSelectionChange"
          style="width: 100%"
          class="optimized-table"
          :default-sort = "{prop: 'time', order: 'descending'}"
      >
        <el-table-column type="selection" width="55" align="center"></el-table-column>
        <el-table-column
            prop="id"
            label="序号"
            width="80"
            align="center"
            sortable
        ></el-table-column>

        <el-table-column
            prop="time"
            label="上报时间"
            min-width="180"
            :formatter="formatTime"
        ></el-table-column>

        <!-- 数值列统一右对齐并添加单位 -->
        <el-table-column prop="t" label="温度(℃)" min-width="110" align="right">
          <template slot-scope="{row}">
            {{ Number(row.t).toFixed(1) }}
          </template>
        </el-table-column>

        <el-table-column prop="h" label="湿度(%)" min-width="110" align="right">
          <template slot-scope="{row}">
            {{ Number(row.h).toFixed(1) }}
          </template>
        </el-table-column>

        <el-table-column prop="waterlevel" label="水位" min-width="120" align="right">
          <template slot-scope="{row}">
            {{ Math.round(row.waterlevel) }}
          </template>
        </el-table-column>

        <el-table-column prop="weight" label="重量(g)" min-width="120" align="right">
          <template slot-scope="{row}">
            {{ Math.round(row.weight) }}
          </template>
        </el-table-column>
        <el-table-column prop="pump" label="泵状态" min-width="100" align="center">
          <template slot-scope="{row}">
            <span>{{ row.pump ? '开启' : '关闭' }}</span>
          </template>
        </el-table-column>
        <el-table-column prop="fan" label="风扇状态" min-width="100" align="center">
          <template slot-scope="{row}">
            <span>{{ row.fan ? '开启' : '关闭' }}</span>
          </template>
        </el-table-column>
        <el-table-column prop="motor" label="电机状态" min-width="100" align="center">
          <template slot-scope="{row}">
            <span>{{ row.motor ? '开启' : '关闭' }}</span>
          </template>
        </el-table-column>
        <el-table-column prop="light" label="光照状态" min-width="100" align="center">
          <template slot-scope="{row}">
            <span>{{ row.light ? '开启' : '关闭' }}</span>
          </template>
        </el-table-column>
        <el-table-column prop="door" label="圈门状态" min-width="100" align="center">
          <template slot-scope="{row}">
            <span>{{ row.door ? '开启' : '关闭' }}</span>
          </template>
        </el-table-column>
        <el-table-column prop="heat" label="加热状态" min-width="100" align="center">
          <template slot-scope="{row}">
            <span>{{ row.heat ? '开启' : '关闭' }}</span>
          </template>
        </el-table-column>
        <el-table-column prop="lng" label="经度" min-width="120" align="right">
          <template slot-scope="{row}">
            {{ row.lng.toFixed(6) }}
          </template>
        </el-table-column>
        <el-table-column prop="lat" label="纬度" min-width="120" align="right">
          <template slot-scope="{row}">
            {{ row.lat.toFixed(6) }}
          </template>
        </el-table-column>
        <el-table-column prop="altitude" label="海拔(m)" min-width="120" align="right">
          <template slot-scope="{row}">
            {{ row.altitude.toFixed(2) }}
          </template>
        </el-table-column>




      </el-table>

      <!-- 分页区域 -->
      <div class="pagination">
        <el-pagination
            background
            @current-change="handleCurrentChange"
            :current-page="pageNum"
            :page-sizes="[5, 10, 20]"
            :page-size="pageSize"
            layout="total, prev, pager, next"
            :total="total">
        </el-pagination>
      </div>
    </div>
  </div>
</template>

<script>
export default {
  name: "SensorDataHis",
  data() {
    return {
      tableData: [],  // 所有的数据
      pageNum: 1,   // 当前的页码
      pageSize: 10,  // 每页显示的个数
      total: 0,
      title: null,
      fromVisible: false,
      form: {},
      user: JSON.parse(localStorage.getItem('xm-user') || '{}'),
      rules: {
        title: [
          {required: true, message: '请输入标题', trigger: 'blur'},
        ],
        content: [
          {required: true, message: '请输入内容', trigger: 'blur'},
        ]
      },
      ids: [],
      resultVisible:false,
      resultList:[],
    }
  },
  created() {
    this.load(1)
  },
  methods: {
    handleSelectionChange(rows) {   // 当前选中的所有的行数据
      this.ids = rows.map(v => v.id)   //  [1,2]
    },
    delBatch() {   // 批量删除
      if (!this.ids.length) {
        this.$message.warning('请选择数据')
        return
      }
      this.$confirm('您确定批量删除这些数据吗？', '确认删除', {type: "warning"}).then(response => {
        this.$request.delete('/rtsd/delete/batch', {data: this.ids}).then(res => {
          if (res.code === '200') {   // 表示操作成功
            this.$message.success('操作成功')
            this.load(1)
          } else {
            this.$message.error(res.msg)  // 弹出错误的信息
          }
        })
      }).catch(() => {
      })
    },
    load(pageNum) {  // 分页查询
      if (pageNum) this.pageNum = pageNum
      this.$request.get('/rtsd/selectPage', {
        params: {
          pageNum: this.pageNum,
          pageSize: this.pageSize,
          title: this.title,
        }
      }).then(res => {
        this.tableData = res.data?.list
        this.total = res.data?.total
      })
    },
    reset() {
      this.title = null
      this.load(1)
    },
    handleCurrentChange(pageNum) {
      this.load(pageNum)
    },
    formatTime(row, column) {
      // 假设时间格式化方法，根据实际数据格式调整
      if (!row.time) return ''
      return new Date(row.time).toLocaleString()
    }
  }
}
</script>

<style scoped>
/* 容器样式 */
.container {
  padding: 20px;
}

/* 搜索区域样式 - 关键：添加底部外边距拉开与表格的距离 */
.search {
  display: flex;
  align-items: center;
  margin-bottom: 20px; /* 按钮与表格的间距 */
  flex-wrap: wrap; /* 支持换行 */
  gap: 10px; /* 按钮之间的间距 */
}

/* 表格区域样式 */
.table {
  width: 100%;
}

.optimized-table {
  font-size: 14px;
  --el-table-header-bg-color: #f8f9fa;
  --el-table-row-hover-bg-color: #f8fafc;
}

.optimized-table /deep/ .el-table__header th {
  font-weight: 600;
  color: #2c3e50;
}

.optimized-table /deep/ .el-table__cell {
  padding: 10px 0;
}

.optimized-table /deep/ .numeric-cell {
  font-family: 'Roboto Mono', monospace;
}

/* 分页区域样式 */
.pagination {
  display: flex;
  justify-content: flex-end;
  margin-top: 20px;
}

/* 响应式调整 */
@media (max-width: 768px) {
  .table {
    overflow-x: auto;
  }

  .optimized-table {
    min-width: 800px;
  }

  .search {
    margin-bottom: 15px; /* 移动端减小间距 */
  }

  .search .el-input {
    width: 100% !important;
  }
}

.coordinate-detail div {
  font-family: monospace;
  font-size: 13px;
}

.coordinate-range div {
  font-family: monospace;
  font-size: 13px;
  color: #606266;
}
</style>