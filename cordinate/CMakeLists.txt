cmake_minimum_required(VERSION 3.8)
project(cordinate)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 필요한 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
#find_package(sensor_msgs REQUIRED)

include_directories(
  "${PROJECT_SOURCE_DIR}/src"
)
# libserial 찾기
#find_path(LIBSERIAL_INCLUDE_DIR
#  NAMES libserial/SerialPort.h
#  PATHS /usr/include /usr/local/include
#)
#
#find_library(LIBSERIAL_LIB
#  NAMES serial
#  PATHS /usr/lib /usr/local/lib
#)


#if(NOT LIBSERIAL_INCLUDE_DIR OR NOT LIBSERIAL_LIB)
#  message(FATAL_ERROR "libserial not found. Please install libserial-dev.")
#endif()


#include_directories(${LIBSERIAL_INCLUDE_DIR})

include_directories(
  "${PROJECT_SOURCE_DIR}/src"
)

add_executable(cordinate_converter src/cordinate.cpp)

ament_target_dependencies(cordinate_converter rclcpp std_msgs nav_msgs geometry_msgs)

#target_link_libraries(rf_publisher_node ${LIBSERIAL_LIB})

install(TARGETS
  cordinate_converter
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

#install(DIRECTORY launch/
#  DESTINATION share/${PROJECT_NAME}/launch
#)
ament_export_include_directories(include)
ament_package()
