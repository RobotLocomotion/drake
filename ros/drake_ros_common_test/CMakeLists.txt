cmake_minimum_required(VERSION 2.8.3)
project(drake_ros_common_test)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS drake_ros_common roscpp)

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES
  #CATKIN_DEPENDS
  #DEPENDS
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
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

  add_rostest_gtest(
      simulation_abort_function_test
      test/simulation_abort_function_test.test
      test/simulation_abort_function_test.cc)
  target_link_libraries(simulation_abort_function_test
      ${catkin_LIBRARIES})
  set_target_properties(simulation_abort_function_test
      PROPERTIES EXCLUDE_FROM_ALL FALSE)
endif()

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")
