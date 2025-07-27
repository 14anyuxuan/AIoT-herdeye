package com.example.controller;

import com.example.MessageDataHolder;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestHeader;
import org.springframework.web.bind.annotation.RestController;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.util.Map;

@RestController
public class IoTHttpController {
    @PostMapping("/iot-endpoint")
    public String handleIoTMessage(
            @RequestHeader Map<String, String> headers,
            @RequestBody String body) {

        System.out.println("Received Headers:");
        headers.forEach((key, value) ->
                System.out.println(key + " = " + value));

        System.out.println("Received Body: " + body);
        printEnvValues(body);
//打印日志，解析，写代码推到前端
        return "{\"status\": \"success\", \"message\": \"Data received\"}";
    }
    private void printEnvValues(String body) {
        try {
            ObjectMapper mapper = new ObjectMapper();
            JsonNode root = mapper.readTree(body);
            String detailsStr = root.path("Details").asText();
            JsonNode params = mapper.readTree(detailsStr).path("Params");

            // 解析浮点字段（使用double）
            double humidity = params.path("environment:H").path("Value").asDouble();
            double temperature = params.path("environment:T").path("Value").asDouble();
            double longitude = params.path("GPS:lng").path("Value").asDouble();
            double latitude = params.path("GPS:lat").path("Value").asDouble();
            double altitude = params.path("GPS:altitude").path("Value").asDouble();
            double cogt = params.path("GPS:cogt").path("Value").asDouble();
            double speed = params.path("GPS:speed").path("Value").asDouble();

            // 解析整数字段（使用int）
            int water_level = params.path("home:water_level").path("Value").asInt();
            int weight = params.path("home:weight").path("Value").asInt();
            int pump = params.path("home:pump").path("Value").asInt();
            int motor = params.path("home:motor").path("Value").asInt();
            int light = params.path("home:light").path("Value").asInt();
            int door = params.path("home:door").path("Value").asInt();

            // 使用正确的类型更新存储
            MessageDataHolder.updateProperty("H", humidity);
            MessageDataHolder.updateProperty("T", temperature);
            MessageDataHolder.updateProperty("lng", longitude);
            MessageDataHolder.updateProperty("lat", latitude);
            MessageDataHolder.updateProperty("altitude", altitude);
            MessageDataHolder.updateProperty("cogt", cogt);
            MessageDataHolder.updateProperty("speed", speed);
            MessageDataHolder.updateProperty("water_level", water_level);
            MessageDataHolder.updateProperty("weight", weight);
            MessageDataHolder.updateProperty("pump", pump);
            MessageDataHolder.updateProperty("motor", motor);
            MessageDataHolder.updateProperty("light", light);
            MessageDataHolder.updateProperty("door", door);

            System.out.printf(">>> 解析结果：湿度H = %.2f%%   温度T = %.2f℃   经度lng = %.6f  纬度lat = %.6f%n  海拔altitude = %.6f  真北航向角cogt = %.6f  速度speed = %.2f  水位water_level = %d 重量weight = %d%n  水槽水泵pump=%d  割草机电机motor=%d  光照light=%d  圈门door=%d%n",
                    humidity, temperature, longitude, latitude, altitude, cogt, speed,
                    water_level, weight, pump, motor, light, door);

        } catch (Exception e) {
            System.err.println("解析物联网数据失败: " + e.getMessage());
            e.printStackTrace();
        }
    }
}
