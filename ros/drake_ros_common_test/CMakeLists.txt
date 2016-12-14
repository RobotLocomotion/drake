cmake_minimum_required(VERSION 3.5)
project(drake_ros_common_test)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS drake_ros_common roscpp)

catkin_package()

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  # The inclusion of ${CMAKE_INSTALL_PREFIX}/include is necessary to ensure the
  # correct version of gtest is used.
  include_directories(
      ${CMAKE_INSTALL_PREFIX}/include
      ${catkin_INCLUDE_DIRS})

  add_rostest_gtest(
      parameter_server_test
      test/parameter_server_test.test
      test/parameter_server_test.cc)
  target_link_libraries(parameter_server_test
      ${catkin_LIBRARIES})
  set_target_properties(parameter_server_test PROPERTIES EXCLUDE_FROM_ALL FALSE)
endif()

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")
