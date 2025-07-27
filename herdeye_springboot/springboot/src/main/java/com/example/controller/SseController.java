package com.example.controller;

import com.example.MessageDataHolder;
import org.springframework.scheduling.annotation.Scheduled;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;
import org.springframework.web.servlet.mvc.method.annotation.SseEmitter;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.IOException;
import java.util.Map;
import java.util.UUID;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ThreadLocalRandom;

@RestController
@RequestMapping("/sse")
public class SseController {
    private final Map<String, SseEmitter> emitters = new ConcurrentHashMap<>();
    private static final Logger log = LoggerFactory.getLogger(SseController.class);

    // 前端订阅入口
    @GetMapping("/subscribe")
    public SseEmitter subscribe() {
        SseEmitter emitter = new SseEmitter(3600_000L); // 超时时间
        String clientId = UUID.randomUUID().toString();
        emitters.put(clientId, emitter);

        emitter.onCompletion(() -> emitters.remove(clientId));
        emitter.onTimeout(() -> emitters.remove(clientId));

        return emitter;
    }

    // 主动推送数据的方法
    public void pushData(String data) {
        emitters.forEach((id, emitter) -> {
            try {
                emitter.send(SseEmitter.event().data(data));
            } catch (Exception e) {
                log.warn("Failed to send to emitter [{}]: {}", id, e.toString());
                emitter.complete();
                emitters.remove(id);
            }
        });
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

        // 创建JSON对象（避免字符串拼接错误）
        String jsonData = String.format(
                "{\"T\": %.2f, \"H\": %.2f, \"lng\": %.6f, \"lat\": %.6f, " +
                        "\"altitude\": %.6f, \"cogt\": %.6f, \"speed\": %.2f, " +
                        "\"water_level\": %d, \"weight\": %d, \"pump\": %d, " +
                        "\"motor\": %d, \"light\": %d, \"door\": %d}",
                T.doubleValue(), H.doubleValue(), lng.doubleValue(), lat.doubleValue(),
                altitude.doubleValue(), cogt.doubleValue(), speed.doubleValue(),
                water_level.intValue(), weight.intValue(), pump.intValue(),
                motor.intValue(), light.intValue(), door.intValue()
        );

        log.info("push: {}", jsonData);
        pushData(jsonData);
    }
}
