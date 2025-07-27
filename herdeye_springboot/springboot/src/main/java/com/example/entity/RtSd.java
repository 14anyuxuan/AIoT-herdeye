package com.example.entity;

import java.io.Serializable;

/**
 * 公告信息表
*/
public class RtSd implements Serializable {
    private static final long serialVersionUID = 1L;

    /**
     * ID
     */
    private Integer id;
    private String time;
    private Double lat;
    private Double lng;
    private Double altitude;
    private Double cogt;
    private Double speed;
    private Double T;
    private Double H;
    private Integer water_level;
    private Integer weight;
    private Integer pump;
    private Integer motor;
    private Integer light;
    private Integer door;

    public Integer getId() {
        return id;
    }

    public void setId(Integer id) {
        this.id = id;
    }

    public String getTime() {
        return time;
    }

    public void setTime(String time) {
        this.time = time;
    }

    public Double getLat() {return lat;}

    public void setLat(Double lat) {this.lat = lat;}

    public Double getLng() {return lng;}

    public void setLng(Double lng) {this.lng = lng;}

    public Double getAltitude() {return altitude;}

    public void setAltitude(Double altitude) {this.altitude = altitude;}

    public Double getCogt() {return cogt;}

    public void setCogt(Double cogt) {this.cogt = cogt;}

    public Double getSpeed() {return speed;}

    public void setSpeed(Double speed) {this.speed = speed;}

    public Double getT() {return T;}

    public void setT(Double t) {T = t;}

    public Double getH() {return H;}

    public void setH(Double h) {H = h;}

    public Integer getWater_level() {return water_level;}

    public void setWater_level(Integer water_level) {this.water_level = water_level;}

    public Integer getWeight() { return weight; }

    public void setWeight(Integer weight) { this.weight = weight; }

    public Integer getMotor() { return motor; }

    public void setMotor(Integer motor) { this.motor = motor; }

    public Integer getPump() { return pump; }

    public void setPump(Integer pump) { this.pump = pump; }

    public Integer getLight() { return light; }

    public void setLight(Integer light) { this.light = light; }

    public Integer getDoor() { return door; }

    public void setDoor(Integer door) { this.door = door; }
}