cmake_minimum_required(VERSION 3.5)
project(drake_ros_systems)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
    drake_ros_common
    roscpp
    sensor_msgs
    tf)

# Include Drake's version of Eigen.
set(EIGEN3_INCLUDE_DIRS ${CMAKE_INSTALL_PREFIX}/include/eigen3)
set(EIGEN3_FOUND true)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

# Specifies additional locations of header files.
include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include
)

link_directories("${CMAKE_INSTALL_PREFIX}/lib")

add_library(${PROJECT_NAME} SHARED
    src/ros_clock_publisher.cc
    src/ros_tf_publisher.cc)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    drakeSystemFramework)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  include_directories(${CMAKE_INSTALL_PREFIX}/include)

  add_rostest_gtest(ros_test test/ros_test.test test/ros_test.cc)
  target_link_libraries(ros_test ${catkin_LIBRARIES})
  set_target_properties(ros_test PROPERTIES EXCLUDE_FROM_ALL FALSE)

  add_rostest_gtest(ros_tf_publisher_test test/ros_tf_publisher_test.test
      test/ros_tf_publisher_test.cc
      src/ros_tf_publisher.cc)
  target_link_libraries(ros_tf_publisher_test
      ${catkin_LIBRARIES}
      drakeCommon
      drakeJoints
      drakeMultibodyParsers
      drakeRBM
      drakeSystemFramework)
  set_target_properties(ros_tf_publisher_test PROPERTIES EXCLUDE_FROM_ALL FALSE)

  add_rostest_gtest(ros_clock_publisher_test test/ros_clock_publisher_test.test
      test/ros_clock_publisher_test.cc
      src/ros_clock_publisher.cc)
  target_link_libraries(ros_clock_publisher_test
      ${catkin_LIBRARIES}
      drakeCommon
      drakeSystemFramework)
  set_target_properties(ros_clock_publisher_test PROPERTIES
      EXCLUDE_FROM_ALL FALSE)
endif()

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")
