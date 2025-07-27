package com.example.service;

import com.example.entity.Uav;
import com.example.mapper.UavMapper;
import com.github.pagehelper.PageHelper;
import com.github.pagehelper.PageInfo;
import jakarta.annotation.Resource;
import org.springframework.stereotype.Service;

import java.util.List;

/**
 * 公告信息表业务处理
 **/
@Service
public class UavService {

    @Resource
    private UavMapper uavMapper;

    /**
     * 新增
     */
    public void add(Uav uav) {
        uavMapper.insert(uav);
    }

    /**
     * 删除
     */
    public void deleteById(Integer id) {
        uavMapper.deleteById(id);
    }

    /**
     * 批量删除
     */
    public void deleteBatch(List<Integer> ids) {
        for (Integer id : ids) {
            uavMapper.deleteById(id);
        }
    }

    /**
     * 修改
     */
    public void updateById(Uav uav) {
        uavMapper.updateById(uav);
    }

    /**
     * 根据ID查询
     */
    public Uav selectById(Integer id) {
        return uavMapper.selectById(id);
    }

    /**
     * 查询所有
     */
    public List<Uav> selectAll(Uav uav) {
        return uavMapper.selectAll(uav);
    }

    /**
     * 分页查询
     */
    public PageInfo<Uav> selectPage(Uav uav, Integer pageNum, Integer pageSize) {
        PageHelper.startPage(pageNum, pageSize);
        List<Uav> list = uavMapper.selectAll(uav);
        return PageInfo.of(list);
    }

}