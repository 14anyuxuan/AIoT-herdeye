package com.example.controller;

import com.example.common.Result;
import com.example.entity.Uav;
import com.example.service.UavService;
import com.github.pagehelper.PageInfo;
import jakarta.annotation.Resource;
import org.springframework.web.bind.annotation.*;

import java.util.List;

/**
 * 公告信息表前端操作接口
 **/
@RestController
@RequestMapping("/uav")
public class UavController {

    @Resource
    private UavService uavService;

    /**
     * 新增
     */
    @PostMapping("/add")
    public Result add(@RequestBody Uav uav) {
        uavService.add(uav);
        return Result.success();
    }

    /**
     * 删除
     */
    @DeleteMapping("/delete/{id}")
    public Result deleteById(@PathVariable Integer id) {
        uavService.deleteById(id);
        return Result.success();
    }

    /**
     * 批量删除
     */
    @DeleteMapping("/delete/batch")
    public Result deleteBatch(@RequestBody List<Integer> ids) {
        uavService.deleteBatch(ids);
        return Result.success();
    }

    /**
     * 修改
     */
    @PutMapping("/update")
    public Result updateById(@RequestBody Uav uav) {
        uavService.updateById(uav);
        return Result.success();
    }

    /**
     * 根据ID查询
     */
    @GetMapping("/selectById/{id}")
    public Result selectById(@PathVariable Integer id) {
        Uav uav = uavService.selectById(id);
        return Result.success(uav);
    }

    /**
     * 查询所有
     */
    @GetMapping("/selectAll")
    public Result selectAll(Uav uav ) {
        List<Uav> list = uavService.selectAll(uav);
        return Result.success(list);
    }

    /**
     * 分页查询
     */
    @GetMapping("/selectPage")
    public Result selectPage(Uav uav,
                             @RequestParam(defaultValue = "1") Integer pageNum,
                             @RequestParam(defaultValue = "10") Integer pageSize) {
        PageInfo<Uav> page = uavService.selectPage(uav, pageNum, pageSize);
        return Result.success(page);
    }

}