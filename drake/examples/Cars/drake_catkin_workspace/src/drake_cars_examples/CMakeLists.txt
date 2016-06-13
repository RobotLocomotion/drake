cmake_minimum_required(VERSION 2.8.3)
project(drake_cars_examples)

if(CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
endif()

## Finds the required macros and libraries.
find_package(catkin REQUIRED COMPONENTS roscpp tf sensor_msgs)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES controlit
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(golfcart_adapter src/golfcart_adapter.cc)
target_link_libraries(golfcart_adapter ${catkin_LIBRARIES})
