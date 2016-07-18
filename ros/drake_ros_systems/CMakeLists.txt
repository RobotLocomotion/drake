cmake_minimum_required(VERSION 2.8.3)
project(drake_ros_systems)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES
  #CATKIN_DEPENDS
  #DEPENDS
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  include_directories(${CMAKE_INSTALL_PREFIX}/include)
  add_rostest_gtest(ros_test test/ros_test.test test/ros_test.cc)
  target_link_libraries(ros_test ${catkin_LIBRARIES})
endif()

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")
