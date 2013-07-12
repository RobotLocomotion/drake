cmake_minimum_required(VERSION 2.6.0) 
#cmake_minimum_required(VERSION 2.8.8)  # need the object library

# set up pods
set(POD_NAME drake)
include(cmake/pods.cmake)

if (UNIX)

# install pkg-config file for gurobi
# todo: move gurobi to it's own (wrapper) pod
set(gurobi_pc ${PKG_CONFIG_OUTPUT_PATH}/gurobi.pc)
file(WRITE ${gurobi_pc}
           "prefix=${CMAKE_CURRENT_SOURCE_DIR}/thirdParty/gurobi/linux64\n"
	   "exec_prefix=\${prefix}\n"
	   "includedir=\${prefix}/include\n"
	   "libdir=\${exec_prefix}/lib\n"
	   "\n"
	   "Name: gurobi\n"
	   "Description: Gurobi Optimizer\n"
	   "Version: 5.1\n"
	   "Cflags: -I\${includedir}\n"
	   "Libs: -L\${libdir} -lgurobi51\n"
	   )
endif()
install(FILES ${gurobi_pc} DESTINATION lib/pkgconfig)

# set up and build lcm types
include(cmake/lcmtypes.cmake)
lcmtypes_build()  

# set up matlab build
include(cmake/mex.cmake)

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

message(STATUS "Writing addpath_drake.m and rmpath_drake.m to ${CMAKE_INSTALL_PREFIX}/matlab") 
file(WRITE ${CMAKE_INSTALL_PREFIX}/matlab/addpath_drake.m
	   "function addpath_drake()\n"
	   "  wd = cd('${CMAKE_CURRENT_SOURCE_DIR}');\n"
	   "  addpath_drake();\n"
	   "  cd(wd);\n"
    )

file(WRITE ${CMAKE_INSTALL_PREFIX}/matlab/rmpath_drake.m
	   "function addpath_drake()\n"
	   "  wd = cd('${CMAKE_CURRENT_SOURCE_DIR}');\n"
	   "  rmpath_drake();\n"
	   "  cd(wd);\n"
    )


# todo: build doxygen here (and remove it from the Makefile)
# http://majewsky.wordpress.com/2010/08/14/tip-of-the-day-cmake-and-doxygen/


