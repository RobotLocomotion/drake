cmake_minimum_required(VERSION 3.5.1)
project(drake)

include(../cmake/config.cmake)
include(../cmake/install.cmake)
include(../cmake/packages.cmake)
include(../cmake/matlab.cmake)

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

# tinydir (via third_party)
add_library(tinydir ../third_party/com_github_cxong_tinydir/tinydir.h)
target_include_directories(tinydir PUBLIC ../third_party/com_github_cxong_tinydir)
set_target_properties(tinydir PROPERTIES LINKER_LANGUAGE CXX)
drake_install_libraries(tinydir)

# spruce (via third_party)
add_library(spruce
  ../third_party/josephdavisco_spruce/spruce.hh
  ../third_party/josephdavisco_spruce/spruce.cc)
target_include_directories(spruce
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../third_party/josephdavisco_spruce>)
drake_install_libraries(spruce)

# stx (via third_party)
add_library(stx ../third_party/com_github_tcbrindle_cpp17_headers/stx/optional.hpp)
target_include_directories(stx PUBLIC ../third_party/com_github_tcbrindle_cpp17_headers)
set_target_properties(stx PROPERTIES LINKER_LANGUAGE CXX)
drake_install_libraries(stx)
install(FILES ../third_party/com_github_tcbrindle_cpp17_headers/stx/optional.hpp
  DESTINATION "include/stx/stx/optional.hpp")

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
