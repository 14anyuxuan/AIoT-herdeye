package com.example.mapper;

import com.example.entity.Sensor;

import java.util.List;

/**
 * 操作sensor相关数据接口
*/
public interface SensorMapper {

    /**
      * 新增
    */
    int insert(Sensor sensor);

    /**
      * 删除
    */
    int deleteById(Integer id);

    /**
      * 修改
    */
    int updateById(Sensor sensor);

    /**
      * 根据ID查询
    */
    Sensor selectById(Integer id);

    /**
      * 查询所有
    */
    List<Sensor> selectAll(Sensor sensor);

}