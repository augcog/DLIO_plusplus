###########################################################
#                                                         #
# Copyright (c)                                           #
#                                                         #
# The Verifiable & Control-Theoretic Robotics (VECTR) Lab #
# University of California, Los Angeles                   #
#                                                         #
# Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez   #
# Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu         #
#                                                         #
###########################################################

cmake_minimum_required(VERSION 3.16)
project(gicp_plusplus)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenMP REQUIRED)
find_package(LibXml2 REQUIRED)  # URDF parsing for offline aux-LiDAR extrinsics (lidar_concat)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)

# Not all machines have <cpuid.h> available
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# rosidl_get_typesupport_target(cpp_typesupport_target
#   ${PROJECT_NAME} rosidl_typesupport_cpp)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(include)
include_directories(SYSTEM)

# small_gicp is vendored as a header-only dependency for this parallel package.
add_library(gicp_plusplus_small_gicp INTERFACE)
target_include_directories(gicp_plusplus_small_gicp INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/small_gicp/include>
)
target_link_libraries(gicp_plusplus_small_gicp INTERFACE Eigen3::Eigen OpenMP::OpenMP_CXX)

# Localization Node
add_executable(gicp_plusplus_node src/localization_node.cc src/localization.cc)
target_include_directories(gicp_plusplus_node PRIVATE ${LIBXML2_INCLUDE_DIR})
target_link_libraries(gicp_plusplus_node
  ${PCL_LIBRARIES}
  Threads::Threads
  OpenMP::OpenMP_CXX
  gicp_plusplus_small_gicp
  ${LIBXML2_LIBRARIES}
)
ament_target_dependencies(gicp_plusplus_node "tf2_ros" "tf2_geometry_msgs" "tf2_eigen" "tf2_sensor_msgs" "pcl_ros" "nav_msgs" "pcl_conversions" "rclcpp" "sensor_msgs" "geometry_msgs" "visualization_msgs")

ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs sensor_msgs geometry_msgs visualization_msgs pcl_ros)

# Binaries
install( TARGETS gicp_plusplus_node
         DESTINATION lib/${PROJECT_NAME}
)
install( PROGRAMS
         scripts/profile_localization_resources.py
         scripts/plot_localization_profile.py
         DESTINATION lib/${PROJECT_NAME}
)
install( DIRECTORY cfg launch
         DESTINATION share/${PROJECT_NAME}
)
# Install the vehicle URDF alongside the package share so the launch's walk-up
# (share/gicp_plusplus/launch -> share/gicp_plusplus) resolves av24.urdf
# in the colcon install tree, not just the source tree. OPTIONAL: the source-tree
# walk-up / static fallback still apply if the file isn't present at install time.
install( FILES av24.urdf
         DESTINATION share/${PROJECT_NAME}
         OPTIONAL
)

link_directories(${PCL_LIBRARY_DIRS})

file(WRITE ${CMAKE_BINARY_DIR}/test_cpuid.cpp "#include <cpuid.h>")
try_compile(HAS_CPUID ${CMAKE_BINARY_DIR} ${CMAKE_BINARY_DIR}/test_cpuid.cpp)
file(REMOVE ${CMAKE_BINARY_DIR}/test_cpuid.cpp)
if(HAS_CPUID)
  add_compile_definitions(HAS_CPUID)
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(
    luminar_sweep_matching_test
    test/luminar_sweep_matching_test.cpp
  )
  target_include_directories(luminar_sweep_matching_test PRIVATE include)
  ament_add_gtest(
    rtk_gate_test
    test/rtk_gate_test.cpp
  )
  target_include_directories(rtk_gate_test PRIVATE include)
  ament_add_gtest(
    imu_range_test
    test/imu_range_test.cpp
  )
  target_include_directories(imu_range_test PRIVATE include)
endif()

ament_package()
