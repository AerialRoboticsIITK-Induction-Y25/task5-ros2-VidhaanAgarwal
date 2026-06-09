cmake_minimum_required(VERSION 3.10)
project(part1_drone_fleet)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(drone_fleet_demo
  src/vehicle.cpp
  src/drone.cpp
  src/mission_drone.cpp
  src/autonomous_drone.cpp
  src/main.cpp
)

target_include_directories(drone_fleet_demo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)