# -*- mode: cmake -*-
# vi: set ft=cmake :

cmake_minimum_required(VERSION 3.5.1)
project(drake)

list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake/modules")

include(CTest)

configure_file(CTestCustom.cmake.in CTestCustom.cmake @ONLY)

if(CYGWIN OR NOT UNIX)
  message(FATAL_ERROR "Cygwin and non-Unix platforms are NOT supported")
endif()

find_package(Bazel 0.6.1 MODULE REQUIRED)

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

find_package(PythonInterp 2.7 EXACT MODULE REQUIRED)

if(CMAKE_BUILD_TYPE STREQUAL Debug)
  set(BAZEL_COMPILATION_MODE dbg)
else()
  set(BAZEL_COMPILATION_MODE opt)
endif()

set(BAZEL_ENV)

if(NOT APPLE)
  foreach(LANG C CXX)
    set(COMPILER ${CMAKE_${LANG}_COMPILER})
    configure_file(tools/cc_toolchain/wrapper.sh.in ${LANG}_wrapper.sh @ONLY)
  endforeach()

  list(APPEND BAZEL_ENV
    "CC=${CMAKE_CURRENT_BINARY_DIR}/C_wrapper.sh"
    "CXX=${CMAKE_CURRENT_BINARY_DIR}/CXX_wrapper.sh"
  )
endif()

configure_file(tools/cc_toolchain/bazel.rc.in bazel.rc @ONLY)

set(BAZEL_STARTUP_ARGS "--bazelrc=${CMAKE_CURRENT_BINARY_DIR}/bazel.rc")

set(BAZEL_ARGS)

if(CMAKE_VERBOSE_MAKEFILE)
  list(APPEND BAZEL_ARGS --subcommands)
endif()

option(WITH_GUROBI "Build with support for Gurobi" OFF)

if(WITH_GUROBI)
  find_package(Gurobi 7.5.2 MODULE REQUIRED)

  list(APPEND BAZEL_ARGS --config=gurobi)

  if(NOT APPLE)
    get_filename_component(GUROBI_PATH "${Gurobi_INCLUDE_DIRS}" DIRECTORY)
    list(APPEND BAZEL_ENV "GUROBI_PATH=${GUROBI_PATH}")
  endif()
endif()

option(WITH_MATLAB "Build with support for MATLAB" OFF)

if(WITH_MATLAB)
  list(APPEND BAZEL_ARGS --define=NO_IPOPT=ON)
endif()

option(WITH_MOSEK "Build with support for MOSEK" OFF)

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
  SOURCE_DIR "${PROJECT_SOURCE_DIR}"
  CONFIGURE_COMMAND :
  BUILD_COMMAND ${BAZEL_ENV} "${Bazel_EXECUTABLE}" ${BAZEL_STARTUP_ARGS} build ${BAZEL_ARGS} ${BAZEL_TARGETS}
  BUILD_IN_SOURCE 1
  BUILD_ALWAYS 1
  INSTALL_COMMAND ${BAZEL_ENV} "${Bazel_EXECUTABLE}" ${BAZEL_STARTUP_ARGS} run ${BAZEL_ARGS} ${BAZEL_TARGETS} -- "${CMAKE_INSTALL_PREFIX}"
  USES_TERMINAL_BUILD 1
  USES_TERMINAL_INSTALL 1
)

list(APPEND CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX}")

if(WITH_MATLAB)
  ExternalProject_Add(drake_matlab
    DEPENDS drake_cxx_python
    SOURCE_DIR "${PROJECT_SOURCE_DIR}/matlab"
    CMAKE_ARGS
      -DBUILD_TESTING:BOOL=${BUILD_TESTING}
      -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
      "-DCMAKE_CXX_COMPILER:FILEPATH=${CMAKE_CXX_COMPILER}"
      "-DCMAKE_CXX_FLAGS:STRING=${CMAKE_CXX_FLAGS}"
      "-DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}"
      "-DCMAKE_PREFIX_PATH:PATH=${CMAKE_PREFIX_PATH}"
      "-DCMAKE_SHARED_LINKER_FLAGS:STRING=${CMAKE_SHARED_LINKER_FLAGS}"
      -DCMAKE_VERBOSE_MAKEFILE:BOOL=${CMAKE_VERBOSE_MAKEFILE}
    BINARY_DIR "${PROJECT_BINARY_DIR}/drake_matlab-build"
    BUILD_ALWAYS 1
  )
endif()
