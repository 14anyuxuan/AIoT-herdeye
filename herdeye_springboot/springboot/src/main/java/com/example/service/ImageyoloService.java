package com.example.service;

import com.example.entity.Imageyolo;
import com.example.mapper.ImageyoloMapper;
import com.github.pagehelper.PageHelper;
import com.github.pagehelper.PageInfo;
import jakarta.annotation.Resource;
import org.springframework.stereotype.Service;

import java.util.List;

/**
 * 公告信息表业务处理
 **/
@Service
public class ImageyoloService {

    @Resource
    private ImageyoloMapper imageyoloMapper;

    /**
     * 新增
     */
    public void add(Imageyolo imageyolo) {
        imageyoloMapper.insert(imageyolo);
    }

    /**
     * 删除
     */
    public void deleteById(Integer id) {
        imageyoloMapper.deleteById(id);
    }

    /**
     * 批量删除
     */
    public void deleteBatch(List<Integer> ids) {
        for (Integer id : ids) {
            imageyoloMapper.deleteById(id);
        }
    }

    /**
     * 修改
     */
    public void updateById(Imageyolo imageyolo) {
        imageyoloMapper.updateById(imageyolo);
    }

    /**
     * 根据ID查询
     */
    public Imageyolo selectById(Integer id) {
        return imageyoloMapper.selectById(id);
    }

    /**
     * 查询所有
     */
    public List<Imageyolo> selectAll(Imageyolo imageyolo) {
        return imageyoloMapper.selectAll(imageyolo);
    }

    /**
     * 分页查询
     */
    public PageInfo<Imageyolo> selectPage(Imageyolo imageyolo, Integer pageNum, Integer pageSize) {
        PageHelper.startPage(pageNum, pageSize);
        List<Imageyolo> list = imageyoloMapper.selectAll(imageyolo);
        return PageInfo.of(list);
    }

}