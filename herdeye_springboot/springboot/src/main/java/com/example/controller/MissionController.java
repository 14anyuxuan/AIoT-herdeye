package com.example.controller;

import com.example.common.Result;
import com.example.entity.Mission;
import com.example.service.MissionService;
import com.github.pagehelper.PageInfo;
import jakarta.annotation.Resource;
import org.springframework.web.bind.annotation.*;

import java.util.List;

/**
 * 公告信息表前端操作接口
 **/
@RestController
@RequestMapping("/mission")
public class MissionController {

    @Resource
    private MissionService missionService;

    /**
     * 新增
     */
    @PostMapping("/add")
    public Result add(@RequestBody Mission mission) {
        missionService.add(mission);
        return Result.success();
    }

    /**
     * 删除
     */
    @DeleteMapping("/delete/{id}")
    public Result deleteById(@PathVariable Integer id) {
        missionService.deleteById(id);
        return Result.success();
    }

    /**
     * 批量删除
     */
    @DeleteMapping("/delete/batch")
    public Result deleteBatch(@RequestBody List<Integer> ids) {
        missionService.deleteBatch(ids);
        return Result.success();
    }

    /**
     * 修改
     */
    @PutMapping("/update")
    public Result updateById(@RequestBody Mission mission) {
        missionService.updateById(mission);
        return Result.success();
    }

    /**
     * 根据ID查询
     */
    @GetMapping("/selectById/{id}")
    public Result selectById(@PathVariable Integer id) {
        Mission mission = missionService.selectById(id);
        return Result.success(mission);
    }

    /**
     * 查询所有
     */
    @GetMapping("/selectAll")
    public Result selectAll(Mission mission ) {
        List<Mission> list = missionService.selectAll(mission);
        return Result.success(list);
    }

    /**
     * 分页查询
     */
    @GetMapping("/selectPage")
    public Result selectPage(Mission mission,
                             @RequestParam(defaultValue = "1") Integer pageNum,
                             @RequestParam(defaultValue = "10") Integer pageSize) {
        PageInfo<Mission> page = missionService.selectPage(mission, pageNum, pageSize);
        return Result.success(page);
    }

}