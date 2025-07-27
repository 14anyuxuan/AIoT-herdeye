package com.example.entity;

import java.io.Serializable;

/**
 * 公告信息表
*/
public class Imageyolo implements Serializable {
    private static final long serialVersionUID = 1L;

    /** ID */
    private Integer imageId;
    /** 标题 */
    private Integer missionIdim;

    /** 创建时间 */
    private String detectionTime;
    /** 创建人 */
    private String srcImage;
    private String yoloImage;
    private String detectionResult;
    private String yoloModule;

    public Integer getImageId() {
        return imageId;
    }

    public void setImageId(Integer imageId) {
        this.imageId = imageId;
    }

    public Integer getMissionIdim() {
        return missionIdim;
    }

    public void setMissionIdim(Integer missionIdim) {
        this.missionIdim = missionIdim;
    }

    public String getDetectionTime() {
        return detectionTime;
    }

    public void setDetectionTime(String detectionTime) {
        this.detectionTime = detectionTime;
    }

    public String getSrcImage() {
        return srcImage;
    }

    public void setSrcImage(String srcImage) {
        this.srcImage = srcImage;
    }

    public String getYoloImage() {
        return yoloImage;
    }

    public void setYoloImage(String yoloImage) {
        this.yoloImage = yoloImage;
    }

    public String getDetectionResult() {
        return detectionResult;
    }

    public void setDetectionResult(String detectionResult) {
        this.detectionResult = detectionResult;
    }

    public String getYoloModule() {
        return yoloModule;
    }

    public void setYoloModule(String yoloModule) {
        this.yoloModule = yoloModule;
    }
}