package com.example.service;

import com.example.entity.Mission;
import com.example.mapper.MissionMapper;
import com.github.pagehelper.PageHelper;
import com.github.pagehelper.PageInfo;
import jakarta.annotation.Resource;
import org.springframework.stereotype.Service;

import java.util.List;

/**
 * 公告信息表业务处理
 **/
@Service
public class MissionService {

    @Resource
    private MissionMapper missionMapper;

    /**
     * 新增
     */
    public void add(Mission mission) {
        missionMapper.insert(mission);
    }

    /**
     * 删除
     */
    public void deleteById(Integer id) {
        missionMapper.deleteById(id);
    }

    /**
     * 批量删除
     */
    public void deleteBatch(List<Integer> ids) {
        for (Integer id : ids) {
            missionMapper.deleteById(id);
        }
    }

    /**
     * 修改
     */
    public void updateById(Mission mission) {
        missionMapper.updateById(mission);
    }

    /**
     * 根据ID查询
     */
    public Mission selectById(Integer id) {
        return missionMapper.selectById(id);
    }

    /**
     * 查询所有
     */
    public List<Mission> selectAll(Mission mission) {
        return missionMapper.selectAll(mission);
    }

    /**
     * 分页查询
     */
    public PageInfo<Mission> selectPage(Mission mission, Integer pageNum, Integer pageSize) {
        PageHelper.startPage(pageNum, pageSize);
        List<Mission> list = missionMapper.selectAll(mission);
        return PageInfo.of(list);
    }

}