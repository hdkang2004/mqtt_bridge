cmake_minimum_required(VERSION 3.8)
project(mqtt_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ublox_msgs REQUIRED)

add_executable(gps_to_mqtt_node src/gps_to_mqtt_node.cpp)

ament_target_dependencies(gps_to_mqtt_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  ublox_msgs
)

target_link_libraries(gps_to_mqtt_node
  mosquitto
)

install(TARGETS
  gps_to_mqtt_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
