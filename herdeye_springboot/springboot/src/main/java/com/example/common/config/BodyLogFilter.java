package com.example.common.config;

import jakarta.servlet.*;
import jakarta.servlet.http.*;
import org.springframework.stereotype.Component;
import org.springframework.web.util.ContentCachingRequestWrapper;
import org.springframework.web.util.ContentCachingResponseWrapper;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import lombok.extern.slf4j.Slf4j;

@Slf4j
public class BodyLogFilter implements Filter {

    @Override
    public void doFilter(ServletRequest req, ServletResponse res, FilterChain chain)
            throws IOException, ServletException {

        HttpServletRequest request = (HttpServletRequest) req;
        HttpServletResponse response = (HttpServletResponse) res;

        // 包装器：用于缓存请求和响应体
        ContentCachingRequestWrapper requestWrapper = new ContentCachingRequestWrapper(request);
        ContentCachingResponseWrapper responseWrapper = new ContentCachingResponseWrapper(response);

        try {
            chain.doFilter(requestWrapper, responseWrapper);
        } finally {
            // 请求体
            String requestBody = getStringValue(requestWrapper.getContentAsByteArray());
            // 响应体
            String responseBody = getStringValue(responseWrapper.getContentAsByteArray());

            log.info("\n[{}] {}\n>>> Request Body: {}\n<<< Response Body: {}",
                    request.getMethod(), request.getRequestURI(), requestBody, responseBody);

            // 响应体必须重新写回客户端，否则前端拿不到内容
            responseWrapper.copyBodyToResponse();
        }
    }

    private String getStringValue(byte[] content) {
        try {
            return new String(content, StandardCharsets.UTF_8);
        } catch (Exception e) {
            return "[unreadable]";
        }
    }
}
