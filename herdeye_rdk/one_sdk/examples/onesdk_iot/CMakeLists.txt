project(onesdk_iot C)
cmake_minimum_required(VERSION 3.10)

# 设置 cJSON 路径
set(CJSON_DIR ${CMAKE_SOURCE_DIR}/external_libs/cJson)
set(CJSON_SRC ${CJSON_DIR}/cJSON.c)
set(CJSON_INC ${CJSON_DIR})

# 检查 cJSON 源文件是否存在
if(NOT EXISTS ${CJSON_SRC})
    message(FATAL_ERROR "cJSON source not found at: ${CJSON_SRC}")
endif()

include_directories(
    ${PROJECT_BINARY_DIR}/build/include
    ${PROJECT_BINARY_DIR}/build/external_libs/libwebsockets/include
    ${CJSON_INC}
    /opt/homebrew/include
)

add_executable(${PROJECT_NAME} onesdk_iot.c ${CJSON_SRC})
target_link_libraries(${PROJECT_NAME} PRIVATE 
    onesdk_shared
    websockets_shared
    m
)

set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD 99)
set_property(TARGET ${PROJECT_NAME} PROPERTY C_STANDARD_REQUIRED ON)