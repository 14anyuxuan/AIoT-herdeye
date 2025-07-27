package com.example.service;

import com.example.entity.Sensor;
import com.example.mapper.SensorMapper;
import com.github.pagehelper.PageHelper;
import com.github.pagehelper.PageInfo;
import jakarta.annotation.Resource;
import org.springframework.stereotype.Service;

import java.util.List;

/**
 * 公告信息表业务处理
 **/
@Service
public class SensorService {

    @Resource
    private SensorMapper sensorMapper;

    /**
     * 新增
     */
    public void add(Sensor sensor) {
        sensorMapper.insert(sensor);
    }

    /**
     * 删除
     */
    public void deleteById(Integer id) {
        sensorMapper.deleteById(id);
    }

    /**
     * 批量删除
     */
    public void deleteBatch(List<Integer> ids) {
        for (Integer id : ids) {
            sensorMapper.deleteById(id);
        }
    }

    /**
     * 修改
     */
    public void updateById(Sensor sensor) {
        sensorMapper.updateById(sensor);
    }

    /**
     * 根据ID查询
     */
    public Sensor selectById(Integer id) {
        return sensorMapper.selectById(id);
    }

    /**
     * 查询所有
     */
    public List<Sensor> selectAll(Sensor sensor) {
        return sensorMapper.selectAll(sensor);
    }

    /**
     * 分页查询
     */
    public PageInfo<Sensor> selectPage(Sensor sensor, Integer pageNum, Integer pageSize) {
        PageHelper.startPage(pageNum, pageSize);
        List<Sensor> list = sensorMapper.selectAll(sensor);
        return PageInfo.of(list);
    }

}