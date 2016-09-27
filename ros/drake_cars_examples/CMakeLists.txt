cmake_minimum_required(VERSION 3.5)
project(drake_cars_examples)

set(CMAKE_CXX_STANDARD 14)

## Finds the required macros and libraries.
find_package(catkin REQUIRED COMPONENTS roscpp tf sensor_msgs drake_ros_systems)


# Assuming drake is being built in the same workspace, which builds eigen into
# the CMAKE_INSTALL_PREFIX
#SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_INSTALL_PREFIX}/share/cmake)
#find_package(Eigen3 3.1.2 REQUIRED)
set(EIGEN3_INCLUDE_DIR ${CMAKE_INSTALL_PREFIX}/include/eigen3)
set(EIGEN3_FOUND true)
#find_package(drake REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_INSTALL_PREFIX}/include
  ${CMAKE_INSTALL_PREFIX}/include/lcmtypes
  )

add_executable(golfcart_adapter src/golfcart_adapter.cc)
target_link_libraries(golfcart_adapter ${catkin_LIBRARIES})

link_directories("${CMAKE_INSTALL_PREFIX}/lib")
add_executable(single_car_in_stata_garage src/single_car_in_stata_garage.cc)
target_link_libraries(single_car_in_stata_garage
  ${catkin_LIBRARIES}
  drakeAutomotiveLcm
  drakeCommon
  drakeRBM
  drakeRBSystem
  drakeLCMSystem
  drakeShapes
  drakeJoints
  drakeRigidBodyConstraint
  lcm)
