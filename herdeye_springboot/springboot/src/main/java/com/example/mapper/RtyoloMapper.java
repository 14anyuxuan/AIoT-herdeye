package com.example.mapper;

import com.example.entity.Rtyolo;

import java.util.List;

/**
 * 操作rtyolo相关数据接口
*/
public interface RtyoloMapper {

    /**
      * 新增
    */
    int insert(Rtyolo rtyolo);

    /**
      * 删除
    */
    int deleteById(Integer id);

    /**
      * 修改
    */
    int updateById(Rtyolo rtyolo);

    /**
      * 根据ID查询
    */
    Rtyolo selectById(Integer id);

    /**
      * 查询所有
    */
    List<Rtyolo> selectAll(Rtyolo rtyolo);

}