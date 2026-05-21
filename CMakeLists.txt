cmake_minimum_required(VERSION 3.16)
project(vicon_px4)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp        REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs      REQUIRED)
find_package(Eigen3        REQUIRED)

add_executable(vicon_px4_node src/vicon_px4_node.cpp)

target_include_directories(vicon_px4_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS})

ament_target_dependencies(vicon_px4_node
  rclcpp
  geometry_msgs
  px4_msgs)

install(TARGETS vicon_px4_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)

ament_package()