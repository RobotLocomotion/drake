cmake_minimum_required(VERSION 2.8.3)
project(drake_ros_common)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES
  #CATKIN_DEPENDS
  #DEPENDS
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")
