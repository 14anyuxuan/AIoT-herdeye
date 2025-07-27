package com.example.service;

import cn.hutool.core.date.DateUtil;
import com.example.entity.Account;
import com.example.entity.Worker;
import com.example.mapper.WorkerMapper;
import com.example.utils.TokenUtils;
import com.github.pagehelper.PageHelper;
import com.github.pagehelper.PageInfo;
import jakarta.annotation.Resource;
import org.springframework.stereotype.Service;

import java.util.List;

/**
 * 公告信息表业务处理
 **/
@Service
public class WorkerService {

    @Resource
    private WorkerMapper workerMapper;

    /**
     * 新增
     */
    public void add(Worker worker) {
        workerMapper.insert(worker);
    }

    /**
     * 删除
     */
    public void deleteById(Integer id) {
        workerMapper.deleteById(id);
    }

    /**
     * 批量删除
     */
    public void deleteBatch(List<Integer> ids) {
        for (Integer id : ids) {
            workerMapper.deleteById(id);
        }
    }

    /**
     * 修改
     */
    public void updateById(Worker worker) {
        workerMapper.updateById(worker);
    }

    /**
     * 根据ID查询
     */
    public Worker selectById(Integer id) {
        return workerMapper.selectById(id);
    }

    /**
     * 查询所有
     */
    public List<Worker> selectAll(Worker worker) {
        return workerMapper.selectAll(worker);
    }

    /**
     * 分页查询
     */
    public PageInfo<Worker> selectPage(Worker worker, Integer pageNum, Integer pageSize) {
        PageHelper.startPage(pageNum, pageSize);
        List<Worker> list = workerMapper.selectAll(worker);
        return PageInfo.of(list);
    }

}