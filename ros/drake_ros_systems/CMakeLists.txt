cmake_minimum_required(VERSION 3.5)
project(drake_ros_systems)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES
  #CATKIN_DEPENDS
  #DEPENDS
)

## Specifies additional locations of header files.
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  include_directories(${CMAKE_INSTALL_PREFIX}/include)
  add_rostest_gtest(ros_test test/ros_test.test test/ros_test.cc)
  target_link_libraries(ros_test ${catkin_LIBRARIES})
  set_target_properties(ros_test PROPERTIES EXCLUDE_FROM_ALL FALSE)
endif()

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")
