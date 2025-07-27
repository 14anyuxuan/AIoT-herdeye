package com.example.mapper;

import com.example.entity.Imageyolo;

import java.util.List;

/**
 * 操作imageyolo相关数据接口
*/
public interface ImageyoloMapper {

    /**
      * 新增
    */
    int insert(Imageyolo imageyolo);

    /**
      * 删除
    */
    int deleteById(Integer id);

    /**
      * 修改
    */
    int updateById(Imageyolo imageyolo);

    /**
      * 根据ID查询
    */
    Imageyolo selectById(Integer id);

    /**
      * 查询所有
    */
    List<Imageyolo> selectAll(Imageyolo imageyolo);

}