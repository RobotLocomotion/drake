cmake_minimum_required(VERSION 2.6.0) 
#cmake_minimum_required(VERSION 2.8.8)  # need the object library

# set up pods
set(POD_NAME drake)
include(cmake/pods.cmake)

# set up and build lcm types
include(cmake/lcmtypes.cmake)
lcmtypes_build()  

# set up matlab build
include(cmake/mex.cmake)

find_package(Java)
include(UseJava)

pods_use_pkg_config_classpath(lcm-java)

add_jar(drake
	util/MyLCMTypeDatabase.java
	util/MessageMonitor.java
	util/CoordinateFrameData.java
	util/LCMCoder.java
	util/Transform.java
	)
install_jar(drake share/java)

pods_install_pkg_config_file(drake-java
    CLASSPATH drake
    REQUIRES lcm-java lcmtypes_drake-java
    VERSION 0.0.1)

pods_find_pkg_config(eigen3)

if (eigen3_FOUND)
   pods_use_pkg_config_includes( eigen3 )
endif()

add_subdirectory(util)
add_subdirectory(algorithms)
add_subdirectory(systems)
add_subdirectory(thirdParty/GetFullPath)

# setup matlab pods-compliance
include(cmake/matlab_pods.cmake)
pods_configure_matlab_paths()

message(STATUS "Writing addpath_drake.m and rmpath_drake.m to ${CMAKE_INSTALL_PREFIX}/matlab") 
file(WRITE ${CMAKE_INSTALL_PREFIX}/matlab/addpath_drake.m
	   "function addpath_drake()\n"
	   "  wd = cd('${CMAKE_CURRENT_SOURCE_DIR}');\n"
	   "  addpath_drake();\n"
	   "  cd(wd);\n"
    )

file(WRITE ${CMAKE_INSTALL_PREFIX}/matlab/rmpath_drake.m
	   "function rmpath_drake()\n"
	   "  wd = cd('${CMAKE_CURRENT_SOURCE_DIR}');\n"
	   "  rmpath_drake();\n"
	   "  cd(wd);\n"
    )


# todo: build doxygen here (and remove it from the Makefile)
# http://majewsky.wordpress.com/2010/08/14/tip-of-the-day-cmake-and-doxygen/


