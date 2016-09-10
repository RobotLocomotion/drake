cmake_minimum_required(VERSION 2.8.3)
project(drake_ros_common)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  #DEPENDS
)

## Specifies additional locations of header files.
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

file(GLOB ${PROJECT_NAME}_SRCS src/*.cc)
add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SRCS})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")
