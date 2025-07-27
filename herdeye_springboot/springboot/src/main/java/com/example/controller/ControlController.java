package com.example.controller;

import com.example.common.Result;
import com.example.entity.Control;
import com.example.service.ControlService;
import com.github.pagehelper.PageInfo;
import jakarta.annotation.Resource;
import org.springframework.web.bind.annotation.*;

import java.util.List;

/**
 * 公告信息表前端操作接口
 **/
@RestController
@RequestMapping("/control")
public class ControlController {

    @Resource
    private ControlService controlService;

    /**
     * 新增
     */
    @PostMapping("/add")
    public Result add(@RequestBody Control control) {
        controlService.add(control);
        return Result.success();
    }

    /**
     * 删除
     */
    @DeleteMapping("/delete/{id}")
    public Result deleteById(@PathVariable Integer id) {
        controlService.deleteById(id);
        return Result.success();
    }

    /**
     * 批量删除
     */
    @DeleteMapping("/delete/batch")
    public Result deleteBatch(@RequestBody List<Integer> ids) {
        controlService.deleteBatch(ids);
        return Result.success();
    }

    /**
     * 修改
     */
    @PutMapping("/update")
    public Result updateById(@RequestBody Control control) {
        controlService.updateById(control);
        return Result.success();
    }

    /**
     * 根据ID查询
     */
    @GetMapping("/selectById/{id}")
    public Result selectById(@PathVariable Integer id) {
        Control control = controlService.selectById(id);
        return Result.success(control);
    }

    /**
     * 查询所有
     */
    @GetMapping("/selectAll")
    public Result selectAll(Control control ) {
        List<Control> list = controlService.selectAll(control);
        return Result.success(list);
    }

    /**
     * 分页查询
     */
    @GetMapping("/selectPage")
    public Result selectPage(Control control,
                             @RequestParam(defaultValue = "1") Integer pageNum,
                             @RequestParam(defaultValue = "10") Integer pageSize) {
        PageInfo<Control> page = controlService.selectPage(control, pageNum, pageSize);
        return Result.success(page);
    }

}