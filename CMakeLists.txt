cmake_minimum_required(VERSION 3.5)
project(object_detection)

# C++ 표준 (필요시 14 이상으로 설정)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# ament_cmake 및 의존 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclpy)

# scan_processor_node: 레이저 스캔 전처리 및 장애물 후보 생성
add_executable(scan_processor_node src/scan_processor_node.cpp)
ament_target_dependencies(scan_processor_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

# obstacle_detector_node: 장애물 후보를 받아 추적 수행
add_executable(obstacle_detector_node src/obstacle_detector_node.cpp)
ament_target_dependencies(obstacle_detector_node
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
)

# visualization_node: 추적 결과를 MarkerArray로 시각화
add_executable(visualization_node src/visualization_node.cpp)
ament_target_dependencies(visualization_node
  rclcpp
  nav_msgs
  visualization_msgs
)

# time_debugger_node: 각 토픽별 time stamp 비교
add_executable(delay_monitor_node src/delay_monitor_node.cpp)
ament_target_dependencies(delay_monitor_node
  rclcpp
  sensor_msgs
  nav_msgs
  message_filters
  geometry_msgs
)

# 설치 대상 설정
install(TARGETS
  scan_processor_node
  obstacle_detector_node
  visualization_node
  delay_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/tracker.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/state_machine.py
  DESTINATION lib/${PROJECT_NAME}
)

# launch 파일이나 기타 리소스 디렉토리도 설치할 경우
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
