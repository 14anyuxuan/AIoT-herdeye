package com.example.mapper;

import com.example.entity.RtSd;

import java.util.List;

/**
 * 操作rtsd相关数据接口
*/
public interface RtSdMapper {

    /**
      * 新增
    */
    int insert(RtSd rtsd);

    /**
      * 删除
    */
    int deleteById(Integer id);

    /**
      * 修改
    */
    int updateById(RtSd rtsd);

    /**
      * 根据ID查询
    */
    RtSd selectById(Integer id);

    /**
      * 查询所有
    */
    List<RtSd> selectAll(RtSd rtsd);

    //新添（好像无用）
    RtSd selectLatest();
}
