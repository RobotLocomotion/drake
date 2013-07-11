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

find_package(Java)
include(UseJava)

execute_process(COMMAND pkg-config --variable=classpath lcm-java OUTPUT_VARIABLE LCM_JAR_FILE)
string(STRIP ${LCM_JAR_FILE} LCM_JAR_FILE)
set( CMAKE_JAVA_INCLUDE_PATH ${LCM_JAR_FILE}:${LCMTYPES_JAR} )

add_jar(drake
	util/MyLCMTypeDatabase.java
	util/MessageMonitor.java
	util/CoordinateFrameData.java
	util/LCMCoder.java
	util/Transform.java
	)
install_jar(drake share/java)

include_directories( ${CMAKE_SOURCE_DIR}/thirdParty/eigen3 ${CMAKE_SOURCE_DIR}/util )

add_subdirectory(util)
add_subdirectory(algorithms)
add_subdirectory(systems)
add_subdirectory(thirdParty/GetFullPath)

# setup matlab pods-compliance
include(cmake/matlab_pods.cmake)
pods_configure_matlab_paths()

file(WRITE $ENV{BUILD_PREFIX}/matlab/addpath_drake.m
	   "function addpath_drake()\n"
	   "  wd = cd ${CMAKE_CURRENT_SOURCE_DIR};\n"
	   "  addpath_drake();\n"
	   "  cd(wd);\n"
    )

file(WRITE $ENV{BUILD_PREFIX}/matlab/rmpath_drake.m
	   "function addpath_drake()\n"
	   "  wd = cd ${CMAKE_CURRENT_SOURCE_DIR};\n"
	   "  rmpath_drake();\n"
	   "  cd(wd);\n"
    )


# todo: build doxygen here (and remove it from the Makefile)
# http://majewsky.wordpress.com/2010/08/14/tip-of-the-day-cmake-and-doxygen/


