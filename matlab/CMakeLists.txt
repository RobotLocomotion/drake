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

if(UNIX AND NOT APPLE)
  # On Linux, force targets to use system Protobuf shared library instead of
  # the shared library bundled with MATLAB. On macOS, the two shared libraries
  # are correctly marked as ABI incompatible.

  get_filename_component(MEX_LIBRARY_DIR "${Matlab_MEX_LIBRARY}" DIRECTORY)
  get_filename_component(PROTOBUF_LIBRARY_DIR
    "${Protobuf_LIBRARY_RELEASE}" DIRECTORY
  )

  set(CMAKE_INSTALL_RPATH
    "$ORIGIN:$ORIGIN/../../../lib:${PROTOBUF_LIBRARY_DIR}:${MEX_LIBRARY_DIR}"
  )

  set(CMAKE_CXX_FLAGS "-Wl,-rpath,${PROTOBUF_LIBRARY_DIR} ${CMAKE_CXX_FLAGS}")
endif()

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
