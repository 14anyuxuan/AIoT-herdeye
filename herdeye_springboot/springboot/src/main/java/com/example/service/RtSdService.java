package com.example.service;

import com.example.entity.RtSd;
import com.example.mapper.RtSdMapper;
import com.github.pagehelper.PageHelper;
import com.github.pagehelper.PageInfo;
import jakarta.annotation.Resource;
import org.springframework.stereotype.Service;

import java.util.List;

/**
 * 公告信息表业务处理
 **/
@Service
public class RtSdService {

    @Resource
    private RtSdMapper rtsdMapper;

    /**
     * 新增
     */
    public void add(RtSd rtsd) {
        rtsdMapper.insert(rtsd);
    }

    /**
     * 删除
     */
    public void deleteById(Integer id) {
        rtsdMapper.deleteById(id);
    }

    /**
     * 批量删除
     */
    public void deleteBatch(List<Integer> ids) {
        for (Integer id : ids) {
            rtsdMapper.deleteById(id);
        }
    }

    /**
     * 修改
     */
    public void updateById(RtSd rtsd) {
        rtsdMapper.updateById(rtsd);
    }

    /**
     * 根据ID查询
     */
    public RtSd selectById(Integer id) {
        return rtsdMapper.selectById(id);
    }

    /**
     * 查询所有
     */
    public List<RtSd> selectAll(RtSd rtsd) {
        return rtsdMapper.selectAll(rtsd);
    }

    /**
     * 分页查询
     */
    public PageInfo<RtSd> selectPage(RtSd rtsd, Integer pageNum, Integer pageSize) {
        PageHelper.startPage(pageNum, pageSize);
        List<RtSd> list = rtsdMapper.selectAll(rtsd);
        return PageInfo.of(list);
    }

    //新添（好像无用）
    public RtSd fetchLatest() {
        return rtsdMapper.selectLatest();
    }

}