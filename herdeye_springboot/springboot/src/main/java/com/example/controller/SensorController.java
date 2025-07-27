package com.example.controller;

import com.example.common.Result;
import com.example.entity.Sensor;
import com.example.service.SensorService;
import com.github.pagehelper.PageInfo;
import jakarta.annotation.Resource;
import org.springframework.web.bind.annotation.*;

import java.util.List;

/**
 * 公告信息表前端操作接口
 **/
@RestController
@RequestMapping("/sensor")
public class SensorController {

    @Resource
    private SensorService sensorService;

    /**
     * 新增
     */
    @PostMapping("/add")
    public Result add(@RequestBody Sensor sensor) {
        sensorService.add(sensor);
        return Result.success();
    }

    /**
     * 删除
     */
    @DeleteMapping("/delete/{id}")
    public Result deleteById(@PathVariable Integer id) {
        sensorService.deleteById(id);
        return Result.success();
    }

    /**
     * 批量删除
     */
    @DeleteMapping("/delete/batch")
    public Result deleteBatch(@RequestBody List<Integer> ids) {
        sensorService.deleteBatch(ids);
        return Result.success();
    }

    /**
     * 修改
     */
    @PutMapping("/update")
    public Result updateById(@RequestBody Sensor sensor) {
        sensorService.updateById(sensor);
        return Result.success();
    }

    /**
     * 根据ID查询
     */
    @GetMapping("/selectById/{id}")
    public Result selectById(@PathVariable Integer id) {
        Sensor sensor = sensorService.selectById(id);
        return Result.success(sensor);
    }

    /**
     * 查询所有
     */
    @GetMapping("/selectAll")
    public Result selectAll(Sensor sensor ) {
        List<Sensor> list = sensorService.selectAll(sensor);
        return Result.success(list);
    }

    /**
     * 分页查询
     */
    @GetMapping("/selectPage")
    public Result selectPage(Sensor sensor,
                             @RequestParam(defaultValue = "1") Integer pageNum,
                             @RequestParam(defaultValue = "10") Integer pageSize) {
        PageInfo<Sensor> page = sensorService.selectPage(sensor, pageNum, pageSize);
        return Result.success(page);
    }

}