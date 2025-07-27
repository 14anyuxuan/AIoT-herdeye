package com.example.entity;

import java.io.Serializable;

/**
 * 公告信息表
*/
public class Sensor implements Serializable {
    private static final long serialVersionUID = 1L;

    /** ID */
    private Integer sensorId;
    /** 标题 */
    private String sensorModel;

    /** 创建时间 */
    private String sensorType;
    /** 创建人 */
    private String sensorPower;

    private String sensorPort;

    private String sensorIntroduce;

    public Integer getSensorId() {
        return sensorId;
    }

    public void setSensorId(Integer sensorId) {
        this.sensorId = sensorId;
    }

    public String getSensorModel() {
        return sensorModel;
    }

    public void setSensorModel(String sensorModel) {
        this.sensorModel = sensorModel;
    }

    public String getSensorType() {
        return sensorType;
    }

    public void setSensorType(String sensorType) {
        this.sensorType = sensorType;
    }

    public String getSensorPower() {
        return sensorPower;
    }

    public void setSensorPower(String sensorPower) {
        this.sensorPower = sensorPower;
    }

    public String getSensorPort() {
        return sensorPort;
    }

    public void setSensorPort(String sensorPort) {
        this.sensorPort = sensorPort;
    }

    public String getSensorIntroduce() {
        return sensorIntroduce;
    }

    public void setSensorIntroduce(String sensorIntroduce) {
        this.sensorIntroduce = sensorIntroduce;
    }
}