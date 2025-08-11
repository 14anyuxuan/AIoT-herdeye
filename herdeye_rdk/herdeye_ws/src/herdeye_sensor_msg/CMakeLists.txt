cmake_minimum_required(VERSION 3.8)
project(herdeye_sensor_msg)


# 设置默认构建类型为Release
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# 生成接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MultiSensorData.msg"
  DEPENDENCIES std_msgs
)

# 安装目录
install(DIRECTORY msg DESTINATION share/${PROJECT_NAME})

# 导出依赖
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(std_msgs)

ament_package()
