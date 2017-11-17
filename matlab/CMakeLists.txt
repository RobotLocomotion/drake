# -*- mode: cmake -*-
# vi: set ft=cmake :

cmake_minimum_required(VERSION 3.5.1)
project(drake_matlab LANGUAGES CXX)

if(CMAKE_VERSION VERSION_LESS 3.10)
  list(INSERT CMAKE_MODULE_PATH 0
    "${PROJECT_SOURCE_DIR}/../third_party/com_kitware_gitlab_cmake_cmake/3.10/Modules"
  )
endif()

include(CTest)

include(cmake/flags.cmake)
include(cmake/functions.cmake)
include(cmake/packages.cmake)

configure_file(addpath_drake.m.in addpath_drake.m @ONLY)

drake_matlab_add_mex(mex_util mex_util.cc mex_util.h)
target_link_libraries(mex_util drake::drake Eigen3::Eigen)

drake_matlab_add_mex(call_matlab_client call_matlab_client.cc)
target_link_libraries(call_matlab_client
  drake::drake mex_util protobuf::libprotobuf
)

install(FILES addpath_drake_install.m DESTINATION share/drake/matlab
  RENAME addpath_drake.m
)

install(TARGETS call_matlab_client mex_util DESTINATION share/drake/matlab)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
