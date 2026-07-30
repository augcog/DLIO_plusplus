cmake_minimum_required(VERSION 3.5.2)
project(gnss_global)

set(CMAKE_CXX_STANDARD 17)

find_package(glim REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(LibXml2 REQUIRED)

if($ENV{ROS_VERSION} EQUAL 2)
  # ROS2
  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()

  ament_auto_add_library(gnss_global SHARED
    src/glim_ext/gnss_global_module_ros2.cpp
  )
elseif($ENV{ROS_VERSION} EQUAL 1)
  # ROS1
  find_package(catkin REQUIRED COMPONENTS glim geometry_msgs nav_msgs)
  catkin_package(
    INCLUDE_DIRS include thirdparty/gtsam_points/include
    LIBRARIES ${glim_LIBRARIES} ${GTSAM_LIBRARIES}
  )

  add_library(gnss_global SHARED
    src/glim_ext/gnss_global_module_ros1.cpp
  )
endif()

target_include_directories(gnss_global PRIVATE
  include
  ${GTSAM_INCLUDE_DIRS}
  ${glim_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${LIBXML2_INCLUDE_DIRS}
)
target_link_libraries(gnss_global
  glim_ext
  ${GTSAM_LIBRARIES}
  ${glim_LIBRARIES}
  spdlog::spdlog
  ${catkin_LIBRARIES}
  ${LIBXML2_LIBRARIES}
)

if($ENV{ROS_VERSION} EQUAL 2 AND BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(gnss_alignment_test
    test/gnss_alignment_test.cpp
  )
  target_include_directories(gnss_alignment_test PRIVATE
    include
  )
  target_link_libraries(gnss_alignment_test Eigen3::Eigen)
endif()
