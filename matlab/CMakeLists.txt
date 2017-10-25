# -*- mode: cmake -*-
# vi: set ft=cmake :

cmake_minimum_required(VERSION 3.5.1)
project(drake_matlab)

if(CMAKE_VERSION VERSION_LESS 3.10)
  list(INSERT CMAKE_MODULE_PATH 0
    "${PROJECT_SOURCE_DIR}/../third_party/com_kitware_gitlab_cmake_cmake/3.10/Modules"
  )
endif()

include(CTest)

set(CMAKE_FIND_PACKAGE_NO_PACKAGE_REGISTRY ON)
set(CMAKE_FIND_PACKAGE_NO_SYSTEM_PACKAGE_REGISTRY ON)

find_package(drake CONFIG REQUIRED)
find_package(Eigen3 3.3.3 CONFIG REQUIRED)
find_package(Matlab 9 MODULE REQUIRED COMPONENTS
  MAIN_PROGRAM MEX_COMPILER MX_LIBRARY
)
find_package(Protobuf 2.6 MODULE REQUIRED)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

configure_file(addpath_drake.m.in addpath_drake.m @ONLY)

install(FILES addpath_drake_install.m DESTINATION share/drake/matlab
  RENAME addpath_drake.m
)

function(drake_matlab_add_mex NAME)
  matlab_add_mex(NAME ${NAME} SHARED SRC ${ARGN})

  set_target_properties(${NAME} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
  )

  foreach(_CONFIGURATION_TYPE ${CMAKE_CONFIGURATION_TYPES})
    string(TOUPPER "${_CONFIGURATION_TYPE}" _CONFIGURATION_TYPE_UPPER)
    set_target_properties(${NAME} PROPERTIES
      LIBRARY_OUTPUT_DIRECTORY_${_CONFIGURATION_TYPE_UPPER}
        "${PROJECT_BINARY_DIR}"
    )
  endforeach()
endfunction()

function(drake_matlab_add_unit_test)
  cmake_parse_arguments("" "" "COMMAND;NAME;SIZE" "" ${ARGN})

  if(NOT _NAME)
    message(FATAL_ERROR
      "The NAME argument to drake_matlab_add_unit_test is required"
    )
  endif()

  string(REPLACE "'" "''" _COMMAND ${_COMMAND})

  set(_CUSTOM_TEST_COMMAND
    "try, addpath_drake; eval('${_COMMAND}'); catch ex, disp(getReport(ex, 'extended')); disp(' '); exit(1); end; exit(0)"
  )

  set(_TEST_ARGS TEST_ARGS ${_UNPARSED_ARGUMENTS})

  matlab_add_unit_test(
    NAME ${_NAME}
    ADDITIONAL_PATH "${CMAKE_CURRENT_SOURCE_DIR};${PROJECT_BINARY_DIR}"
    CUSTOM_TEST_COMMAND "${_CUSTOM_TEST_COMMAND}"
    TIMEOUT -1
    WORKING_DIRECTORY "${PROJECT_BINARY_DIR}"
    ${_TEST_ARGS}
  )

  if(NOT _SIZE)
    set(_SIZE medium)
  endif()

  if(_SIZE STREQUAL small)
    set(_TIMEOUT 60)
  elseif(_SIZE STREQUAL medium)
    set(_TIMEOUT 300)
  elseif(_SIZE STREQUAL large)
    set(_TIMEOUT 900)
  elseif(_SIZE STREQUAL enormous)
    set(_TIMEOUT 2700)
  else()
    message(FATAL_ERROR
      "The SIZE argument to drake_matlab_add_unit_test is invalid"
    )
  endif()

  set(_COST ${_TIMEOUT})
  set(_LABELS ${_SIZE})

  set_tests_properties(${_NAME} PROPERTIES
    COST ${_TIMEOUT}
    LABELS ${_SIZE}
    TIMEOUT ${_TIMEOUT}
  )
endfunction()

add_subdirectory(solvers)
add_subdirectory(util)
