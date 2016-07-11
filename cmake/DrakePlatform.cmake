# Check minimum compiler requirements
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 4.9)
  message(FATAL_ERROR "GCC version must be at least 4.9")
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 7)
  message(FATAL_ERROR "Apple Clang version must be at least 7")
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 3.7)
  message(FATAL_ERROR "Clang version must be at least 3.7")
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 19)
  message(FATAL_ERROR "MSVC version must be at least 19 (VS 2015)")
endif()

# Set default install prefix
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE STRING
    "Prefix for installation of sub-packages (note: required during build!)" FORCE)
endif()
message(STATUS CMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX})

# Require Java at the super-build level since libbot cannot configure without finding Java
find_package(Java 1.6 REQUIRED)
include(UseJava)

# Choose your python (major) version
option(WITH_PYTHON_3 "Force Drake to use python 3 instead of python 2" OFF)
