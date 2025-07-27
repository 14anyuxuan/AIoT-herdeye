package com.example.controller;

import com.example.common.Result;
import com.example.entity.Rtyolo;
import com.example.service.RtyoloService;
import com.github.pagehelper.PageInfo;
import jakarta.annotation.Resource;
import org.springframework.web.bind.annotation.*;

import java.util.List;

/**
 * 公告信息表前端操作接口
 **/
@RestController
@RequestMapping("/rtyolo")
public class RtyoloController {

    @Resource
    private RtyoloService rtyoloService;

    /**
     * 新增
     */
    @PostMapping("/add")
    public Result add(@RequestBody Rtyolo rtyolo) {
        rtyoloService.add(rtyolo);
        return Result.success();
    }

    /**
     * 删除
     */
    @DeleteMapping("/delete/{id}")
    public Result deleteById(@PathVariable Integer id) {
        rtyoloService.deleteById(id);
        return Result.success();
    }

    /**
     * 批量删除
     */
    @DeleteMapping("/delete/batch")
    public Result deleteBatch(@RequestBody List<Integer> ids) {
        rtyoloService.deleteBatch(ids);
        return Result.success();
    }

    /**
     * 修改
     */
    @PutMapping("/update")
    public Result updateById(@RequestBody Rtyolo rtyolo) {
        rtyoloService.updateById(rtyolo);
        return Result.success();
    }

    /**
     * 根据ID查询
     */
    @GetMapping("/selectById/{id}")
    public Result selectById(@PathVariable Integer id) {
        Rtyolo rtyolo = rtyoloService.selectById(id);
        return Result.success(rtyolo);
    }

    /**
     * 查询所有
     */
    @GetMapping("/selectAll")
    public Result selectAll(Rtyolo rtyolo ) {
        List<Rtyolo> list = rtyoloService.selectAll(rtyolo);
        return Result.success(list);
    }

    /**
     * 分页查询
     */
    @GetMapping("/selectPage")
    public Result selectPage(Rtyolo rtyolo,
                             @RequestParam(defaultValue = "1") Integer pageNum,
                             @RequestParam(defaultValue = "10") Integer pageSize) {
        PageInfo<Rtyolo> page = rtyoloService.selectPage(rtyolo, pageNum, pageSize);
        return Result.success(page);
    }

}