package com.example.entity;

import java.io.Serializable;

/**
 * 公告信息表
*/
public class Rtyolo implements Serializable {
    private static final long serialVersionUID = 1L;

    /** ID */
    private Integer RTid;
    /** 标题 */
    private Integer missionIdrt;

    /** 创建时间 */
    private String detectionTime;
    /** 创建人 */
    private String duration;
    private String video;

    private String yoloModule;

    public Integer getRTid() {
        return RTid;
    }

    public void setRTid(Integer RTid) {
        this.RTid = RTid;
    }

    public Integer getMissionIdrt() {
        return missionIdrt;
    }

    public void setMissionIdrt(Integer missionIdrt) {
        this.missionIdrt = missionIdrt;
    }

    public String getDetectionTime() {
        return detectionTime;
    }

    public void setDetectionTime(String detectionTime) {
        this.detectionTime = detectionTime;
    }

    public String getDuration() {
        return duration;
    }

    public void setDuration(String duration) {
        this.duration = duration;
    }

    public String getVideo() {
        return video;
    }

    public void setVideo(String video) {
        this.video = video;
    }

    public String getYoloModule() {
        return yoloModule;
    }

    public void setYoloModule(String yoloModule) {
        this.yoloModule = yoloModule;
    }
}