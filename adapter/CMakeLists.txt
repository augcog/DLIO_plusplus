cmake_minimum_required(VERSION 3.12)
project(adapter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(fusion_engine_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(GeographicLib QUIET)
if(NOT GeographicLib_FOUND)
  list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")
  find_package(GeographicLib REQUIRED)
endif()

add_library(adapter_utils STATIC src/adapter_utils.cpp)
target_include_directories(adapter_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(adapter_utils Eigen3::Eigen)
ament_target_dependencies(adapter_utils
  builtin_interfaces
  fusion_engine_msgs
)

add_executable(adapter src/adapter_node.cpp)
target_include_directories(adapter PRIVATE ${GeographicLib_INCLUDE_DIRS})
target_link_libraries(adapter adapter_utils ${GeographicLib_LIBRARIES})
ament_target_dependencies(adapter
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  fusion_engine_msgs
)

install(TARGETS adapter_utils adapter
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/p1_imu_pcap_replay_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
