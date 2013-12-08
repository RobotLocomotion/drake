cmake_minimum_required(VERSION 2.6.0) 
#cmake_minimum_required(VERSION 2.8.8)  # need the object library

# set up pods
set(POD_NAME spotless)
include(cmake/pods.cmake)

# setup matlab pods-compliance
include(cmake/matlab_pods.cmake)
pods_configure_matlab_paths()

find_program(matlab matlab)
if (NOT matlab )
  message(FATAL_ERROR "Could not find matlab executable")
endif()

add_custom_target(run ALL 
			  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/spot_install_manifest.txt)

add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/spot_install_manifest.txt 
			  COMMAND ${matlab} -nojvm -nosplash -r \"spot_install_manifest\; exit\"
			  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/spotless/spot_install.m
			  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                          )

message(STATUS "Writing addpath_spotless.m and rmpath_spotless.m to ${CMAKE_INSTALL_PREFIX}/matlab") 
file(WRITE ${CMAKE_INSTALL_PREFIX}/matlab/addpath_spotless.m
	   "function addpath_spotless()\n"
	   "  p = importdata(fullfile('${CMAKE_CURRENT_SOURCE_DIR}','spot_install_manifest.txt'));\n"
	   "  addpath(p{:});\n"
    )

file(WRITE ${CMAKE_INSTALL_PREFIX}/matlab/rmpath_spotless.m
	   "function rmpath_spotless()\n"
	   "  p = importdata(fullfile('${CMAKE_CURRENT_SOURCE_DIR}','spot_install_manifest.txt'));\n"
	   "  rmpath(p{:});\n"
    )


