package com.example;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

public class MessageDataHolder {
    // 使用Number类型存储所有数值（支持Integer/Double）
    private static final Map<String, Number> properties = new ConcurrentHashMap<>();

    // 更新整数属性
    public static void updateProperty(String key, int value) {
        properties.put(key, value);
    }

    // 更新浮点属性（重载方法）
    public static void updateProperty(String key, double value) {
        properties.put(key, value);
    }

    // 获取数值（统一返回Number类型）
    public static Number getProperty(String key) {
        return properties.getOrDefault(key, 0);
    }
}
