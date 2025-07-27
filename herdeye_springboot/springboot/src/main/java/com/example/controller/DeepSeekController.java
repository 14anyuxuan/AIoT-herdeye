package com.example.controller;

import org.springframework.ai.chat.client.ChatClient;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RestController;

import java.util.Collections;
import java.util.Map;

@RestController
@RequestMapping("/ai")
public class DeepSeekController {
    private final ChatClient chatclient;

    // 构造方法，用于构造chatclient 实例
    public DeepSeekController(ChatClient.Builder chatClientBuilder) {
        this.chatclient =chatClientBuilder.build();
    }

    @GetMapping("/chat")
    public Map<String, String> chat(@RequestParam String message) {
        String response = chatclient.prompt(message).call().content();

        return Collections.singletonMap("content",  response);  // 返回 {"content":"..."}
    }
}
