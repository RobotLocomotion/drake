# -*- mode: cmake -*-
# vi: set ft=cmake :

cmake_minimum_required(VERSION 3.5.1)
project(drake)

if(CMAKE_VERSION VERSION_LESS 3.7)
  set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules/3.7")
endif()

include(CTest)

find_program(BAZEL_EXECUTABLE bazel)

if(NOT BAZEL_EXECUTABLE)
  message(FATAL_ERROR "Could NOT find Bazel executable")
endif()

execute_process(COMMAND "${BAZEL_EXECUTABLE}" version
  RESULT_VARIABLE EXECUTE_PROCESS_RESULT_VARIABLE
  OUTPUT_VARIABLE EXECUTE_PROCESS_OUTPUT_VARIABLE
)

if(NOT EXECUTE_PROCESS_RESULT_VARIABLE EQUAL 0)
  message(FATAL_ERROR "Could NOT determine Bazel version")
endif()

string(REGEX MATCH "Build label: ([0-9a-zA-Z.]+)"
  REGEX_MATCH_OUTPUT_VARIABLE
  "${EXECUTE_PROCESS_OUTPUT_VARIABLE}"
)

if(CMAKE_MATCH_1 VERSION_LESS 0.6.1)
  message(FATAL_ERROR
    "Bazel version 0.6.1 or higher is required. You are running version ${CMAKE_MATCH_1}"
  )
endif()

get_filename_component(C_COMPILER_REALPATH "${CMAKE_C_COMPILER}" REALPATH)
get_filename_component(C_COMPILER_NAME "${C_COMPILER_REALPATH}" NAME)

get_filename_component(CXX_COMPILER_REALPATH "${CMAKE_CXX_COMPILER}" REALPATH)
get_filename_component(CXX_COMPILER_NAME "${CXX_COMPILER_REALPATH}" NAME)

if(C_COMPILER_NAME STREQUAL ccache OR CXX_COMPILER_NAME STREQUAL ccache)
  message(FATAL_ERROR
    "Compilation with ccache is NOT supported due to incompatibility with Bazel"
  )
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build, options are None Debug Release RelWithDebInfo MinSizeRel"
    FORCE
  )
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY
    STRINGS None Debug Release RelWithDebInfo MinSizeRel
  )
endif()

if(CMAKE_BUILD_TYPE STREQUAL Debug)
  set(BAZEL_COMPILATION_MODE dbg)
else()
  set(BAZEL_COMPILATION_MODE opt)
endif()

set(BAZEL_ENV "CC=${CMAKE_C_COMPILER}" "CXX=${CMAKE_CXX_COMPILER}")

set(BAZEL_ARGS
  --compilation_mode=${BAZEL_COMPILATION_MODE}
  --config=unsupported_crosstool
  --cxxopt=-std=c++14
  --host_cxxopt=-std=c++14
)

if(CMAKE_VERBOSE_MAKEFILE)
  list(APPEND BAZEL_ARGS --subcommands)
endif()

option(WITH_GUROBI "Build with support for GUROBI. Experimental." OFF)

if(WITH_GUROBI)
  list(APPEND BAZEL_ARGS --config=gurobi)
endif()

option(WITH_MOSEK "Build with support for MOSEK. Experimental." OFF)

if(WITH_MOSEK)
  list(APPEND BAZEL_ARGS --config=mosek)
endif()

option(WITH_SNOPT "Build with support for SNOPT" OFF)

if(WITH_SNOPT)
  list(APPEND BAZEL_ARGS --config=snopt)
endif()

set(BAZEL_TARGETS //:install)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "${PROJECT_BINARY_DIR}/install" CACHE STRING
    "Install path prefix, prepended onto install directories" FORCE
  )
endif()

include(ExternalProject)

ExternalProject_Add(drake_cxx_python
  SOURCE_DIR "${PROJECT_SOURCE_DIR}/drake"
  CONFIGURE_COMMAND :
  BUILD_COMMAND ${BAZEL_ENV} "${BAZEL_EXECUTABLE}" build ${BAZEL_ARGS} ${BAZEL_TARGETS}
  BUILD_IN_SOURCE 1
  BUILD_ALWAYS 1
  INSTALL_COMMAND ${BAZEL_ENV} "${BAZEL_EXECUTABLE}" run ${BAZEL_ARGS} ${BAZEL_TARGETS} -- "${CMAKE_INSTALL_PREFIX}"
  USES_TERMINAL_BUILD 1
  USES_TERMINAL_INSTALL 1
)

list(APPEND CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX}")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(WITH_MATLAB "Build with support for MATLAB" OFF)

if(WITH_MATLAB)
  ExternalProject_Add(drake_matlab
    DEPENDS drake_cxx_python
    SOURCE_DIR "${PROJECT_SOURCE_DIR}/drake/matlab"
    CMAKE_ARGS
      -DBUILD_TESTING:BOOL=${BUILD_TESTING}
      -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
      "-DCMAKE_CXX_COMPILER:FILEPATH=${CMAKE_CXX_COMPILER}"
      -DCMAKE_CXX_STANDARD:STRING=${CMAKE_CXX_STANDARD}
      -DCMAKE_CXX_STANDARD_REQUIRED:BOOL=${CMAKE_CXX_STANDARD_REQUIRED}
      "-DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}"
      "-DCMAKE_MODULE_PATH:PATH=${CMAKE_MODULE_PATH}"
      "-DCMAKE_PREFIX_PATH:PATH=${CMAKE_PREFIX_PATH}"
      -DCMAKE_VERBOSE_MAKEFILE:BOOL=${CMAKE_VERBOSE_MAKEFILE}
    BINARY_DIR "${PROJECT_BINARY_DIR}/drake_matlab-build"
    BUILD_ALWAYS 1
  )
endif()
