cmake_minimum_required(VERSION 2.6.0)

# set up pods
set(POD_NAME drake)
include(cmake/pods.cmake)

# set up and build lcm types
include(cmake/lcmtypes.cmake)
lcmtypes_build()  

# set up matlab build
include(cmake/mex.cmake)
mex_setup()

include_directories( ${CMAKE_SOURCE_DIR}/thirdParty/eigen3 ${CMAKE_SOURCE_DIR}/util )

add_subdirectory(util)
#add_subdirectory(algorithms)
add_subdirectory(systems)

