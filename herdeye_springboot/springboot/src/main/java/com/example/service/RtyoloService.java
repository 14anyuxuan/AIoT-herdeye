package com.example.service;

import com.example.entity.Rtyolo;
import com.example.mapper.RtyoloMapper;
import com.github.pagehelper.PageHelper;
import com.github.pagehelper.PageInfo;
import jakarta.annotation.Resource;
import org.springframework.stereotype.Service;

import java.util.List;

/**
 * 公告信息表业务处理
 **/
@Service
public class RtyoloService {

    @Resource
    private RtyoloMapper rtyoloMapper;

    /**
     * 新增
     */
    public void add(Rtyolo rtyolo) {
        rtyoloMapper.insert(rtyolo);
    }

    /**
     * 删除
     */
    public void deleteById(Integer id) {
        rtyoloMapper.deleteById(id);
    }

    /**
     * 批量删除
     */
    public void deleteBatch(List<Integer> ids) {
        for (Integer id : ids) {
            rtyoloMapper.deleteById(id);
        }
    }

    /**
     * 修改
     */
    public void updateById(Rtyolo rtyolo) {
        rtyoloMapper.updateById(rtyolo);
    }

    /**
     * 根据ID查询
     */
    public Rtyolo selectById(Integer id) {
        return rtyoloMapper.selectById(id);
    }

    /**
     * 查询所有
     */
    public List<Rtyolo> selectAll(Rtyolo rtyolo) {
        return rtyoloMapper.selectAll(rtyolo);
    }

    /**
     * 分页查询
     */
    public PageInfo<Rtyolo> selectPage(Rtyolo rtyolo, Integer pageNum, Integer pageSize) {
        PageHelper.startPage(pageNum, pageSize);
        List<Rtyolo> list = rtyoloMapper.selectAll(rtyolo);
        return PageInfo.of(list);
    }

}