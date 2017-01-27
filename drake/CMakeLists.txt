cmake_minimum_required(VERSION 3.5)
project(drake VERSION 0.0.0)

cmake_policy(SET CMP0025 NEW)
cmake_policy(SET CMP0042 NEW)

include(../cmake/config.cmake)
include(../cmake/install.cmake)
include(../cmake/packages.cmake)
include(../cmake/test.cmake)
include(../cmake/tools.cmake)

drake_setup_platform()
drake_setup_tools()
drake_find_packages()

# Create a cache variable that contains the
# max number of concurent link jobs that can be run
# in a Ninja build, if the value is 0 then let Ninja
# use as many as it wants.
set(CMAKE_NINJA_LINK_POOL_SIZE 0 CACHE STRING
  "Number of concurent link jobs that can be run with Ninja build tool")
# if the link pool is non-zero then set the property
# on all the targets in drake
if(CMAKE_NINJA_LINK_POOL_SIZE GREATER 0)
  # Ninja: Configure a job pool to limit simultaneous linking.
  set_property(GLOBAL PROPERTY JOB_POOLS link_pool=${CMAKE_NINJA_LINK_POOL_SIZE})
  # Ninja: Assign all linking to our link job pool.
  set(CMAKE_JOB_POOL_LINK link_pool)
endif()

# options
option(LONG_RUNNING_TESTS "some tests should be run nightly for coverage, but are too slow for CI" OFF)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(BLAS MODULE)
find_package(Doxygen MODULE)
find_package(Perl MODULE)
find_package(Threads MODULE REQUIRED)

# A compatibility shim function that declares a library, with a built-in
# default of SHARED.
function(add_library_with_exports)
  set(options STATIC)
  set(oneValueArgs LIB_NAME)
  set(multiValueArgs SOURCE_FILES)
  cmake_parse_arguments(parsed_args "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  if(parsed_args_STATIC)
    add_library(${parsed_args_LIB_NAME} STATIC ${parsed_args_SOURCE_FILES})
  else()
    add_library(${parsed_args_LIB_NAME} SHARED ${parsed_args_SOURCE_FILES})
  endif()
endfunction()

# This makes all of our #include "drake/..." statemenets work.
include_directories(BEFORE ${PROJECT_SOURCE_DIR}/..)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=all -Werror=ignored-qualifiers -Werror=overloaded-virtual")
if(NOT CMAKE_CXX_FLAGS MATCHES "-fsanitize=") # sanitizers are extremely brittle without lazy linking
  if(APPLE)
    set(CMAKE_SHARED_LINKER_FLAGS "-Wl,-undefined -Wl,error ${CMAKE_SHARED_LINKER_FLAGS}")
    set(CMAKE_MODULE_LINKER_FLAGS "-Wl,-undefined -Wl,error ${CMAKE_MODULE_LINKER_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "-Wl,-undefined -Wl,error ${CMAKE_EXE_LINKER_FLAGS}")
  else()
    set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-undefined ${CMAKE_SHARED_LINKER_FLAGS}")
    set(CMAKE_MODULE_LINKER_FLAGS "-Wl,--no-undefined ${CMAKE_MODULE_LINKER_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "-Wl,--no-undefined ${CMAKE_EXE_LINKER_FLAGS}")
  endif()
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # TODO(#2852) Turn on shadow checking for g++ once we use a version that fixes
  # https://gcc.gnu.org/bugzilla/show_bug.cgi?id=57709
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=shadow")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=inconsistent-missing-override")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=sign-compare")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=return-stack-address")
else()
  # TODO(jwnimmer-tri) Get a similar complement of flags working on clang.
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=extra")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=return-local-addr")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-parameter")
  # TODO(jwnimmer-tri) Fix these warnings and remove this suppression.
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-missing-field-initializers")
endif()
set(CXX_FLAGS_NO_ERROR_SHADOW -Wno-error=shadow -Wno-shadow)
set(CXX_FLAGS_NO_SIGN_COMPARE -Wno-sign-compare)
if(CMAKE_GENERATOR STREQUAL "Xcode")
  # TODO(jamiesnape): Remove when the Xcode CMake generator supports
  # include_directories(SYSTEM) or when the warnings are fixed in Eigen, FCL,
  # and OctoMap.
  set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -Wno-delete-non-virtual-dtor -Wno-shadow")
endif()

# set up matlab build
include(../cmake/mex.cmake)
if(DISABLE_MATLAB AND Matlab_FOUND) # Set by drake_setup_platform
  message(STATUS "MATLAB is disabled because the CMake option DISABLE_MATLAB is set to ON.")
  unset(MATLAB_EXECUTABLE CACHE)
  unset(Matlab_FOUND)
endif()

enable_testing()
include(CTest)
set(GTEST_ROOT ${CMAKE_INSTALL_PREFIX})

set(drake_jar_javafiles matlab/util/Transform.java)
set(drake_jar_requires)


# set up and build lcm types
if(lcm_FOUND)
  add_subdirectory(lcmtypes)

  list(APPEND drake_jar_javafiles
    matlab/util/MyLCMTypeDatabase.java
    matlab/util/MessageMonitor.java
    matlab/util/CoordinateFrameData.java
    matlab/util/LCMCoder.java)
  list(APPEND drake_jar_requires lcm-java lcmtypes_drake-java)

  # add lcmgl
  # libbot does not define a classpath variable. there might be a cleaner way to do this
  if(NOT bot2-lcmgl-client_PREFIX)  # Set by drake_find_package(... PKG_CONFIG)
    message("
  --------------------------------------------------------------------------------
    *** IMPORTANT: bot2-lcmgl build directory not found. ***
  --------------------------------------------------------------------------------
    ")
  else()
    find_jar(LCMGL_JAR_FILE bot2-lcmgl PATHS "${bot2-lcmgl-client_PREFIX}/share/java/")
    if(NOT LCMGL_JAR_FILE)
      message(FATAL_ERROR "
  --------------------------------------------------------------------------------
    *** IMPORTANT: bot2-lcmgl JAR file not found. ***
  --------------------------------------------------------------------------------
      ")
    else()
      message(STATUS "Found bot2-lcmgl")
    endif()
    if(DEFINED CMAKE_JAVA_INCLUDE_PATH)
      set(CMAKE_JAVA_INCLUDE_PATH "${CMAKE_JAVA_INCLUDE_PATH}:${LCMGL_JAR_FILE}")
    else()
      set(CMAKE_JAVA_INCLUDE_PATH "${LCMGL_JAR_FILE}")
    endif()
    list(APPEND drake_jar_javafiles matlab/util/BotLCMGLClient.java)
  endif()
endif()

add_jar(drake SOURCES ${drake_jar_javafiles} INCLUDE_JARS ${drake_jar_requires})
install_jar(drake share/java)
drake_install_pkg_config_file(drake-java
  CLASSPATH drake
  REQUIRES ${drake_jar_requires})

add_subdirectory(thirdParty/bsd/spruce)
add_subdirectory(thirdParty/zlib/tinyxml2)
add_subdirectory(common)
add_subdirectory(lcm)
add_subdirectory(math)
add_subdirectory(multibody)
add_subdirectory(util)
add_subdirectory(solvers)
add_subdirectory(systems)
if(yaml-cpp_FOUND)
  add_subdirectory(automotive)
endif()
if(Matlab_FOUND)
  add_subdirectory(thirdParty/bsd/GetFullPath)
  add_subdirectory(matlab)
endif()
add_subdirectory(examples)

if(NOT DISABLE_PYTHON)
  add_subdirectory(bindings)
  add_subdirectory(pybind)
endif()

if(BUILD_TESTING)
  add_subdirectory(regtests)
endif()

option(ENABLE_DOCUMENTATION "Enable build target for Doxygen and Sphinx documentation." ON)

if(ENABLE_DOCUMENTATION)
  add_subdirectory(doc)
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/doc/textbook/CMakeLists.txt)
  add_subdirectory(doc/textbook)
endif()

drake_install_cmake_package_config_files()
