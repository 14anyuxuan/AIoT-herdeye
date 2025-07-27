package com.example.entity;

import java.io.Serializable;

/**
 * 公告信息表
*/
public class Mission implements Serializable {
    private static final long serialVersionUID = 1L;

    /** ID */
    private Integer missionId;
    /** 标题 */
    private String missionType;

    /** 创建时间 */
    private String missionTime;
    /** 创建人 */
    private String missionPlace;

    /** ID */
    private String workerId;
    /** 标题 */
    private String uavId;

    /** 创建时间 */
    private String sensorId;
    /** 创建人 */
    private String controlId;

    public Integer getMissionId() {
        return missionId;
    }

    public void setMissionId(Integer missionId) {
        this.missionId = missionId;
    }

    public String getMissionType() {
        return missionType;
    }

    public void setMissionType(String missionType) {
        this.missionType = missionType;
    }

    public String getMissionTime() {
        return missionTime;
    }

    public void setMissionTime(String missionTime) {
        this.missionTime = missionTime;
    }

    public String getMissionPlace() {
        return missionPlace;
    }

    public void setMissionPlace(String missionPlace) {
        this.missionPlace = missionPlace;
    }

    public String getWorkerId() {
        return workerId;
    }

    public void setWorkerId(String workerId) {
        this.workerId = workerId;
    }

    public String getUavId() {
        return uavId;
    }

    public void setUavId(String uavId) {
        this.uavId = uavId;
    }

    public String getSensorId() {
        return sensorId;
    }

    public void setSensorId(String sensorId) {
        this.sensorId = sensorId;
    }

    public String getControlId() {
        return controlId;
    }

    public void setControlId(String controlId) {
        this.controlId = controlId;
    }
}