cmake_minimum_required(VERSION 3.10)
project(drone_fleet)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(PART1_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../part1/include)

set(PART1_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../part1/src)
set(PART1_SOURCES
  ${PART1_SRC_DIR}/vehicle.cpp
  ${PART1_SRC_DIR}/drone.cpp
  ${PART1_SRC_DIR}/mission_drone.cpp
  ${PART1_SRC_DIR}/autonomous_drone.cpp
)

add_library(part1_static STATIC ${PART1_SOURCES})
target_include_directories(part1_static PUBLIC ${PART1_INCLUDE_DIR})
set_target_properties(part1_static PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(drone_node src/drone_node.cpp)
target_include_directories(drone_node PRIVATE ${PART1_INCLUDE_DIR})
ament_target_dependencies(drone_node rclcpp std_msgs)
target_link_libraries(drone_node part1_static)

add_executable(fleet_manager src/fleet_manager.cpp)
target_include_directories(fleet_manager PRIVATE ${PART1_INCLUDE_DIR})
ament_target_dependencies(fleet_manager rclcpp std_msgs std_srvs)
target_link_libraries(fleet_manager part1_static)

add_executable(health_monitor src/health_monitor.cpp)
target_include_directories(health_monitor PRIVATE ${PART1_INCLUDE_DIR})
ament_target_dependencies(health_monitor rclcpp std_msgs)
target_link_libraries(health_monitor part1_static)

install(TARGETS
  drone_node
  fleet_manager
  health_monitor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()