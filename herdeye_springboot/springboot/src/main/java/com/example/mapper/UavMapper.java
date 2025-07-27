package com.example.mapper;

import com.example.entity.Uav;

import java.util.List;

/**
 * 操作uav相关数据接口
*/
public interface UavMapper {

    /**
      * 新增
    */
    int insert(Uav uav);

    /**
      * 删除
    */
    int deleteById(Integer id);

    /**
      * 修改
    */
    int updateById(Uav uav);

    /**
      * 根据ID查询
    */
    Uav selectById(Integer id);

    /**
      * 查询所有
    */
    List<Uav> selectAll(Uav uav);

}