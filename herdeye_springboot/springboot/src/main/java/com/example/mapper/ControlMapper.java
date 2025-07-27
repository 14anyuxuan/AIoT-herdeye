package com.example.mapper;

import com.example.entity.Control;

import java.util.List;

/**
 * 操作control相关数据接口
*/
public interface ControlMapper {

    /**
      * 新增
    */
    int insert(Control control);

    /**
      * 删除
    */
    int deleteById(Integer id);

    /**
      * 修改
    */
    int updateById(Control control);

    /**
      * 根据ID查询
    */
    Control selectById(Integer id);

    /**
      * 查询所有
    */
    List<Control> selectAll(Control control);

}