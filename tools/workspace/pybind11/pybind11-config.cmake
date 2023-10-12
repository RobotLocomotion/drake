# This effectively replaces pybind11Common.cmake.
# The targets listed here are correct as of the following list for upstream:
# https://github.com/pybind/pybind11/blob/9c0aa699/tools/pybind11Common.cmake#L8-L13

# Generated by cps2cmake https://github.com/mwoehlke/pycps
# and then subsequently edited by hand.

if(CMAKE_VERSION VERSION_LESS 3.9.0)
  message(FATAL_ERROR "CMake >= 3.9 required")
endif()
cmake_policy(PUSH)
cmake_policy(VERSION 3.0)
set(CMAKE_IMPORT_FILE_VERSION 1)

include(CMakeFindDependencyMacro)

get_filename_component(${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX "${CMAKE_CURRENT_LIST_FILE}" PATH)
get_filename_component(${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}" PATH)
get_filename_component(${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}" PATH)
get_filename_component(${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}" PATH)

if(${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX STREQUAL "/")
  set(${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX)
endif()

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR};${CMAKE_MODULE_PATH}")
find_dependency(PythonInterp MODULE)
find_dependency(PythonLibs MODULE)
set(_expectedTargets pybind11::embed pybind11::module pybind11::pybind11 pybind11::lto pybind11::thin_lto pybind11::python_link_helper pybind11::python2_no_register pybind11::windows_extras pybind11::opt_size)

set(_targetsDefined)
set(_targetsNotDefined)

foreach(_expectedTarget ${_expectedTargets})
  if(NOT TARGET ${_expectedTarget})
    list(APPEND _targetsNotDefined ${_expectedTarget})
  endif()
  if(TARGET ${_expectedTarget})
    list(APPEND _targetsDefined ${_expectedTarget})
  endif()
endforeach()
if("${_targetsDefined}" STREQUAL "${_expectedTargets}")
  set(CMAKE_IMPORT_FILE_VERSION)
  cmake_policy(POP)
  return()
endif()
if(NOT "${_targetsDefined}" STREQUAL "")
  message(FATAL_ERROR "Some (but not all) targets in this export set were already defined.\nTargets Defined: ${_targetsDefined}\nTargets not yet defined: ${_targetsNotDefined}\n")
endif()
unset(_targetsDefined)
unset(_targetsNotDefined)
unset(_expectedTargets)

set(pybind11_VERSION "2.11.1")

add_library(pybind11::embed INTERFACE IMPORTED)
set_target_properties(pybind11::embed PROPERTIES
  INTERFACE_LINK_LIBRARIES "pybind11::pybind11;${${PYTHON_LIBRARIES}_LIBRARIES}"
)

add_library(pybind11::module INTERFACE IMPORTED)
set_target_properties(pybind11::module PROPERTIES
  INTERFACE_LINK_LIBRARIES "pybind11::pybind11;$<$<PLATFORM_ID:Darwin>:-undefined dynamic_lookup>"
)

add_library(pybind11::pybind11 INTERFACE IMPORTED)
set_target_properties(pybind11::pybind11 PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}/include/pybind11;${PYTHON_INCLUDE_DIRS}"
  INTERFACE_COMPILE_FEATURES "cxx_decltype;cxx_decltype_auto"
)

# Below are naively ported targets that are meant to ensure pybind11Tools does
# not fail fast. They may not match what is provided.

add_library(pybind11::lto INTERFACE IMPORTED)
set_target_properties(pybind11::lto PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}/include/pybind11;${PYTHON_INCLUDE_DIRS}"
  INTERFACE_COMPILE_FEATURES "cxx_decltype;cxx_decltype_auto"
)

add_library(pybind11::thin_lto INTERFACE IMPORTED)
set_target_properties(pybind11::thin_lto PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}/include/pybind11;${PYTHON_INCLUDE_DIRS}"
  INTERFACE_COMPILE_FEATURES "cxx_decltype;cxx_decltype_auto"
)

add_library(pybind11::python_link_helper INTERFACE IMPORTED)
set_target_properties(pybind11::python_link_helper PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}/include/pybind11;${PYTHON_INCLUDE_DIRS}"
  INTERFACE_COMPILE_FEATURES "cxx_decltype;cxx_decltype_auto"
)

add_library(pybind11::python2_no_register INTERFACE IMPORTED)
set_target_properties(pybind11::python2_no_register PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}/include/pybind11;${PYTHON_INCLUDE_DIRS}"
  INTERFACE_COMPILE_FEATURES "cxx_decltype;cxx_decltype_auto"
)

add_library(pybind11::windows_extras INTERFACE IMPORTED)
set_target_properties(pybind11::windows_extras PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}/include/pybind11;${PYTHON_INCLUDE_DIRS}"
  INTERFACE_COMPILE_FEATURES "cxx_decltype;cxx_decltype_auto"
)

add_library(pybind11::opt_size INTERFACE IMPORTED)
set_target_properties(pybind11::opt_size PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}/include/pybind11;${PYTHON_INCLUDE_DIRS}"
  INTERFACE_COMPILE_FEATURES "cxx_decltype;cxx_decltype_auto"
)

set(pybind11_LIBRARIES "pybind11::pybind11")
set(pybind11_INCLUDE_DIRS "")

# Simulate `pybind11_strip` per drake-external-examples#216, extracting code from
# https://github.com/pybind/pybind11/blob/v2.7.0/tools/pybind11Common.cmake#L383-L397
# Begin code excerpt.
function(pybind11_strip target_name)
  # Strip unnecessary sections of the binary on Linux/macOS
  if(CMAKE_STRIP)
    if(APPLE)
      set(x_opt -x)
    endif()

    add_custom_command(
      TARGET ${target_name}
      POST_BUILD
      COMMAND ${CMAKE_STRIP} ${x_opt} $<TARGET_FILE:${target_name}>)
  endif()
endfunction()
# End code excerpt.

include(${CMAKE_CURRENT_LIST_DIR}/pybind11Tools.cmake)

unset(${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX)
unset(CMAKE_IMPORT_FILE_VERSION)
cmake_policy(POP)

