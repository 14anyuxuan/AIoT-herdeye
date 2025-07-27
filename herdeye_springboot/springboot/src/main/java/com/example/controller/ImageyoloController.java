package com.example.controller;

import com.example.common.Result;
import com.example.entity.Imageyolo;
import com.example.service.ImageyoloService;
import com.github.pagehelper.PageInfo;
import jakarta.annotation.Resource;
import org.springframework.web.bind.annotation.*;

import java.util.List;

/**
 * 公告信息表前端操作接口
 **/
@RestController
@RequestMapping("/imageyolo")
public class ImageyoloController {

    @Resource
    private ImageyoloService imageyoloService;

    /**
     * 新增
     */
    @PostMapping("/add")
    public Result add(@RequestBody Imageyolo imageyolo) {
        imageyoloService.add(imageyolo);
        return Result.success();
    }

    /**
     * 删除
     */
    @DeleteMapping("/delete/{id}")
    public Result deleteById(@PathVariable Integer id) {
        imageyoloService.deleteById(id);
        return Result.success();
    }

    /**
     * 批量删除
     */
    @DeleteMapping("/delete/batch")
    public Result deleteBatch(@RequestBody List<Integer> ids) {
        imageyoloService.deleteBatch(ids);
        return Result.success();
    }

    /**
     * 修改
     */
    @PutMapping("/update")
    public Result updateById(@RequestBody Imageyolo imageyolo) {
        imageyoloService.updateById(imageyolo);
        return Result.success();
    }

    /**
     * 根据ID查询
     */
    @GetMapping("/selectById/{id}")
    public Result selectById(@PathVariable Integer id) {
        Imageyolo imageyolo = imageyoloService.selectById(id);
        return Result.success(imageyolo);
    }

    /**
     * 查询所有
     */
    @GetMapping("/selectAll")
    public Result selectAll(Imageyolo imageyolo ) {
        List<Imageyolo> list = imageyoloService.selectAll(imageyolo);
        return Result.success(list);
    }

    /**
     * 分页查询
     */
    @GetMapping("/selectPage")
    public Result selectPage(Imageyolo imageyolo,
                             @RequestParam(defaultValue = "1") Integer pageNum,
                             @RequestParam(defaultValue = "10") Integer pageSize) {
        PageInfo<Imageyolo> page = imageyoloService.selectPage(imageyolo, pageNum, pageSize);
        return Result.success(page);
    }

}