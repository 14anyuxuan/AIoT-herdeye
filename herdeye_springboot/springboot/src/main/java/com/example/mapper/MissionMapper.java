package com.example.mapper;

import com.example.entity.Mission;

import java.util.List;

/**
 * 操作mission相关数据接口
*/
public interface MissionMapper {

    /**
      * 新增
    */
    int insert(Mission mission);

    /**
      * 删除
    */
    int deleteById(Integer id);

    /**
      * 修改
    */
    int updateById(Mission mission);

    /**
      * 根据ID查询
    */
    Mission selectById(Integer id);

    /**
      * 查询所有
    */
    List<Mission> selectAll(Mission mission);

}