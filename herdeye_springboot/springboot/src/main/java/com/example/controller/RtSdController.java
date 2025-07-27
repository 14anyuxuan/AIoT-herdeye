package com.example.controller;

import com.example.common.Result;
import com.example.entity.RtSd;
import com.example.service.RtSdService;
import com.github.pagehelper.PageInfo;
import jakarta.annotation.Resource;
import org.springframework.web.bind.annotation.*;

import java.util.List;

/**
 * 公告信息表前端操作接口
 **/
@RestController
@RequestMapping("/rtsd")
public class RtSdController {

    @Resource
    private RtSdService rtsdService;

    /**
     * 新增
     */
    @PostMapping("/add")
    public Result add(@RequestBody RtSd rtsd) {
        rtsdService.add(rtsd);
        return Result.success();
    }

    /**
     * 删除
     */
    @DeleteMapping("/delete/{id}")
    public Result deleteById(@PathVariable Integer id) {
        rtsdService.deleteById(id);
        return Result.success();
    }

    /**
     * 批量删除
     */
    @DeleteMapping("/delete/batch")
    public Result deleteBatch(@RequestBody List<Integer> ids) {
        rtsdService.deleteBatch(ids);
        return Result.success();
    }

    /**
     * 修改
     */
    @PutMapping("/update")
    public Result updateById(@RequestBody RtSd rtsd) {
        rtsdService.updateById(rtsd);
        return Result.success();
    }

    /**
     * 根据ID查询
     */
    @GetMapping("/selectById/{id}")
    public Result selectById(@PathVariable Integer id) {
        RtSd rtsd = rtsdService.selectById(id);
        return Result.success(rtsd);
    }

    /**
     * 查询所有
     */
    @GetMapping("/selectAll")
    public Result selectAll(RtSd rtsd ) {
        List<RtSd> list = rtsdService.selectAll(rtsd);
        return Result.success(list);
    }

    /**
     * 分页查询
     */
    @GetMapping("/selectPage")
    public Result selectPage(RtSd rtsd,
                             @RequestParam(defaultValue = "1") Integer pageNum,
                             @RequestParam(defaultValue = "10") Integer pageSize) {
        PageInfo<RtSd> page = rtsdService.selectPage(rtsd, pageNum, pageSize);
        return Result.success(page);
    }
//新添（无用好像）
    @GetMapping("/latest")
    public Result latest() {
        RtSd latest = rtsdService.fetchLatest();
        return Result.success(latest);
    }

}