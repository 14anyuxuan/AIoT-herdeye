package com.example.entity;

import java.io.Serializable;

/**
 * 公告信息表
*/
public class Uav implements Serializable {
    private static final long serialVersionUID = 1L;

    /** ID */
    private Integer uavId;
    /** 标题 */
    private String uavType;

    /** 创建时间 */
    private String uavModel;
    /** 创建人 */
    private String uavIntroduce;

    public Integer getUavId() {
        return uavId;
    }

    public void setUavId(Integer uavId) {
        this.uavId = uavId;
    }

    public String getUavType() {
        return uavType;
    }

    public void setUavType(String uavType) {
        this.uavType = uavType;
    }

    public String getUavModel() {
        return uavModel;
    }

    public void setUavModel(String uavModel) {
        this.uavModel = uavModel;
    }

    public String getUavIntroduce() {
        return uavIntroduce;
    }

    public void setUavIntroduce(String uavIntroduce) {
        this.uavIntroduce = uavIntroduce;
    }
}