package com.example.service;

import com.example.entity.Control;
import com.example.mapper.ControlMapper;
import com.github.pagehelper.PageHelper;
import com.github.pagehelper.PageInfo;
import jakarta.annotation.Resource;
import org.springframework.stereotype.Service;

import java.util.List;

/**
 * 公告信息表业务处理
 **/
@Service
public class ControlService {

    @Resource
    private ControlMapper controlMapper;

    /**
     * 新增
     */
    public void add(Control control) {
        controlMapper.insert(control);
    }

    /**
     * 删除
     */
    public void deleteById(Integer id) {
        controlMapper.deleteById(id);
    }

    /**
     * 批量删除
     */
    public void deleteBatch(List<Integer> ids) {
        for (Integer id : ids) {
            controlMapper.deleteById(id);
        }
    }

    /**
     * 修改
     */
    public void updateById(Control control) {
        controlMapper.updateById(control);
    }

    /**
     * 根据ID查询
     */
    public Control selectById(Integer id) {
        return controlMapper.selectById(id);
    }

    /**
     * 查询所有
     */
    public List<Control> selectAll(Control control) {
        return controlMapper.selectAll(control);
    }

    /**
     * 分页查询
     */
    public PageInfo<Control> selectPage(Control control, Integer pageNum, Integer pageSize) {
        PageHelper.startPage(pageNum, pageSize);
        List<Control> list = controlMapper.selectAll(control);
        return PageInfo.of(list);
    }

}