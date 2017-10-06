cmake_minimum_required(VERSION 3.5.1)
project(drake)

include(../cmake/config.cmake)
include(../cmake/install.cmake)
include(../cmake/packages.cmake)
include(../cmake/test.cmake)
include(../cmake/mex.cmake)

drake_setup_platform()
drake_find_packages()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# This makes all of our #include "drake/..." statements work.
include_directories(BEFORE ${PROJECT_SOURCE_DIR}/..)

# To find generated protobuf headers.
include_directories(BEFORE ${PROJECT_BINARY_DIR}/..)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=inconsistent-missing-override")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=sign-compare")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=return-stack-address")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=non-virtual-dtor")
else()
  # TODO(jwnimmer-tri) Get a similar complement of flags working on clang.
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=extra")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=return-local-addr")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=non-virtual-dtor")
  # TODO(jwnimmer-tri) Fix these warnings and remove this suppression.
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-missing-field-initializers")
  # This is only intended for tests, but is included for the entire build
  # in CMake since there is not an easy mechanism to enable this only for
  # tests.
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-parameter")
endif()

include(CTest)

add_subdirectory(thirdParty/boost/tcbrindle_cpp17_headers/stx)
add_subdirectory(thirdParty/bsd/spruce)
add_subdirectory(thirdParty/zlib/tinyxml2)
add_subdirectory(common)
add_subdirectory(math)
add_subdirectory(multibody)
add_subdirectory(util)
add_subdirectory(solvers)

add_subdirectory(bindings/python)
add_subdirectory(bindings/pybind11)

add_subdirectory(matlab)

option(ENABLE_DOCUMENTATION "Enable build target for Doxygen and Sphinx documentation." ON)

if(ENABLE_DOCUMENTATION)
  add_subdirectory(doc)
endif()
