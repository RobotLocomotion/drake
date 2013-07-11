cmake_minimum_required(VERSION 2.6.0)

set(POD_NAME drake)
include(cmake/pods.cmake)
include(cmake/lcmtypes.cmake)

lcmtypes_build() 


#add_subdirectory(util)
#add_subdirectory(algorithms)
add_subdirectory(systems)

