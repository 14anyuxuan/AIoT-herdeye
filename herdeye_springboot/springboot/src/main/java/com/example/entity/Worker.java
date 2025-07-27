package com.example.entity;

import java.io.Serializable;

/**
 * 公告信息表
*/
public class Worker implements Serializable {
    private static final long serialVersionUID = 1L;

    /** ID */
    private Integer workerId;
    /** 标题 */
    private String workerName;
    /** 内容 */
    private Integer workerAge;
    /** 创建时间 */
    private String workerPhone;
    /** 创建人 */
    private String workerPlace;

    public Integer getWorkerId() {
        return workerId;
    }

    public void setWorkerId(Integer workerId) {
        this.workerId = workerId;
    }

    public String getWorkerName() {
        return workerName;
    }

    public void setWorkerName(String workerName) {
        this.workerName = workerName;
    }

    public Integer getWorkerAge() {
        return workerAge;
    }

    public void setWorkerAge(Integer workerAge) {
        this.workerAge = workerAge;
    }

    public String getWorkerPhone() {
        return workerPhone;
    }

    public void setWorkerPhone(String workerPhone) {
        this.workerPhone = workerPhone;
    }

    public String getWorkerPlace() {
        return workerPlace;
    }

    public void setWorkerPlace(String workerPlace) {
        this.workerPlace = workerPlace;
    }
}