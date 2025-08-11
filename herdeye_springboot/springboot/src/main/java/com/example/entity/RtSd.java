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
    private Double t;
    private Double h;
    private Integer waterlevel;
    private Integer weight;
    private Integer pump;
    private Integer motor;
    private Integer light;
    private Integer door;
    private Integer heat;
    private Integer fan;

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

    public Double gett() {return t;}

    public void sett(Double t) {this.t = t;}

    public Double geth() {return h;}

    public void seth(Double h) {this.h = h;}

    public Integer getWaterlevel() {return waterlevel;}

    public void setWaterlevel(Integer waterlevel) {this.waterlevel = waterlevel;}

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

    public Integer getHeat() { return heat; }

    public void setHeat(Integer heat) { this.heat = heat; }

    public Integer getFan() { return fan; }

    public void setFan(Integer fan) { this.fan = fan; }
}