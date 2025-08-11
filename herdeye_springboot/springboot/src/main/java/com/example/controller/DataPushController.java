package com.example.controller;

import com.example.MessageDataHolder;
import org.springframework.scheduling.annotation.Scheduled;
import org.springframework.web.bind.annotation.RestController;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Autowired;

@RestController
public class DataPushController {
    private static final Logger log = LoggerFactory.getLogger(DataPushController.class);

    private final DataWebSocketHandler dataWebSocketHandler;

    @Autowired
    public DataPushController(DataWebSocketHandler dataWebSocketHandler) {
        this.dataWebSocketHandler = dataWebSocketHandler;
    }

    @Scheduled(fixedRate = 5000)
    public void autoPush() {
        // 获取所有属性值
        Number T = MessageDataHolder.getProperty("T");
        Number H = MessageDataHolder.getProperty("H");
        Number lng = MessageDataHolder.getProperty("lng");
        Number lat = MessageDataHolder.getProperty("lat");
        Number altitude = MessageDataHolder.getProperty("altitude");
        Number cogt = MessageDataHolder.getProperty("cogt");
        Number speed = MessageDataHolder.getProperty("speed");
        Number water_level = MessageDataHolder.getProperty("water_level");
        Number weight = MessageDataHolder.getProperty("weight");
        Number pump = MessageDataHolder.getProperty("pump");
        Number motor = MessageDataHolder.getProperty("motor");
        Number light = MessageDataHolder.getProperty("light");
        Number door = MessageDataHolder.getProperty("door");
        Number heat = MessageDataHolder.getProperty("heat");
        Number fan = MessageDataHolder.getProperty("fan");

        // 创建JSON对象
        String jsonData = String.format(
                "{\"T\": %.2f, \"H\": %.2f, \"lng\": %.6f, \"lat\": %.6f, " +
                        "\"altitude\": %.6f, \"cogt\": %.6f, \"speed\": %.2f, " +
                        "\"water_level\": %d, \"weight\": %d, \"pump\": %d, " +
                        "\"motor\": %d, \"light\": %d, \"door\": %d, \"heat\": %d, \"fan\": %d}",
                T.doubleValue(), H.doubleValue(), lng.doubleValue(), lat.doubleValue(),
                altitude.doubleValue(), cogt.doubleValue(), speed.doubleValue(),
                water_level.intValue(), weight.intValue(), pump.intValue(),
                motor.intValue(), light.intValue(), door.intValue(), heat.intValue(), fan.intValue()
        );

        log.info("WebSocket push: {}", jsonData);
        dataWebSocketHandler.broadcast(jsonData);
    }
}