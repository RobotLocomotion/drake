cmake_minimum_required(VERSION 3.5)
project(drake_examples_toyota_hsrb)

set(CMAKE_CXX_STANDARD 14)

## Finds the required macros and libraries.
find_package(catkin REQUIRED COMPONENTS
    drake_ros_common
    drake_ros_systems
    roscpp
    tf
    sensor_msgs)

set(EIGEN3_INCLUDE_DIR ${CMAKE_INSTALL_PREFIX}/include/eigen3)
set(EIGEN3_FOUND true)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${CMAKE_INSTALL_PREFIX}/include
    ${CMAKE_INSTALL_PREFIX}/include/lcmtypes)

link_directories("${CMAKE_INSTALL_PREFIX}/lib")

add_executable(passive_demo
    src/passive_demo.cc
    src/passive_demo_common.cc)
target_link_libraries(passive_demo
    ${catkin_LIBRARIES}
    drakeCommon
    drakeHsrb
    drakeJoints
    drakeLcm
    drakeRBM
    drakeRBSystem
    drakeRigidBodyConstraint
    drakeRigidBodyPlant
    drakeShapes
    drakeSystemAnalysis
    drakeSystemFramework
    gflags
    lcm)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(
      passive_demo_test
      test/passive_demo_test.test
      test/passive_demo_test.cc
      src/passive_demo_common.cc)
  target_link_libraries(passive_demo_test
      ${catkin_LIBRARIES}
      drakeCommon
      drakeHsrb
      drakeJoints
      drakeLcm
      drakeRBM
      drakeRBSystem
      drakeRigidBodyConstraint
      drakeRigidBodyPlant
      drakeShapes
      drakeSystemAnalysis
      drakeSystemFramework
      lcm)
  set_target_properties(passive_demo_test
      PROPERTIES EXCLUDE_FROM_ALL FALSE)
endif()
