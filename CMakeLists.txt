cmake_minimum_required(VERSION 2.6.0)

# set up pods
set(POD_NAME drake)
include(cmake/pods.cmake)

# set up and build lcm types
include(cmake/lcmtypes.cmake)
lcmtypes_build()  

# set up matlab build
include(cmake/mex.cmake)
find_matlab()

#add_subdirectory(util)
#add_subdirectory(algorithms)
add_subdirectory(systems)

