package com.example.entity;

import java.io.Serializable;

/**
 * 公告信息表
*/
public class Control implements Serializable {
    private static final long serialVersionUID = 1L;

    /** ID */
    private Integer controlId;
    /** 标题 */
    private String controlType;

    /** 创建时间 */
    private String controlIntroduce;

    public Integer getControlId() {
        return controlId;
    }

    public void setControlId(Integer controlId) {
        this.controlId = controlId;
    }

    public String getControlType() {
        return controlType;
    }

    public void setControlType(String controlType) {
        this.controlType = controlType;
    }

    public String getControlIntroduce() {
        return controlIntroduce;
    }

    public void setControlIntroduce(String controlIntroduce) {
        this.controlIntroduce = controlIntroduce;
    }
}