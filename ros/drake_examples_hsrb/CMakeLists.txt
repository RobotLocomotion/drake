cmake_minimum_required(VERSION 3.5)
project(drake_examples_hsrb)

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
    ${CMAKE_INSTALL_PREFIX}/include/lcmtypes
    )

link_directories("${CMAKE_INSTALL_PREFIX}/lib")

add_executable(demo1 src/demo1.cc)
target_link_libraries(demo1
    ${catkin_LIBRARIES}
    drakeAutomotive
    drakeCommon
    drakeHsrb
    drakeJoints
    drakeLcm
    drakeRBM
    drakeRBSystem
    drakeRigidBodyConstraint
    drakeRigidBodyPlant
    drakeShapes
    drakeSystemFramework
    gflags
    lcm)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(
      demo1_test
      test/demo1_test.test
      test/demo1_test.cc)
  target_link_libraries(demo1_test
      ${catkin_LIBRARIES}
      drakeAutomotive
      drakeCommon
      drakeHsrb
      drakeJoints
      drakeLcm
      drakeRBM
      drakeRBSystem
      drakeRigidBodyConstraint
      drakeRigidBodyPlant
      drakeShapes
      drakeSystemFramework
      lcm)
  set_target_properties(demo1_test
      PROPERTIES EXCLUDE_FROM_ALL FALSE)
endif()
