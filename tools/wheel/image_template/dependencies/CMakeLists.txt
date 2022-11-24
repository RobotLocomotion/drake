cmake_minimum_required(VERSION 3.10.0)
project(pip-drake-dependencies)

# See: https://cmake.org/cmake/help/latest/policy/CMP0135.html
if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()

# Enable checking for `if(APPLE_ARM64)`, needed by some projects.
if(APPLE)
  execute_process(
      COMMAND "/usr/bin/arch"
      RESULT_VARIABLE APPLE_ARCHITECTURE_RESULT_VARIABLE
      OUTPUT_VARIABLE APPLE_ARCHITECTURE
      OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(NOT APPLE_ARCHITECTURE_RESULT_VARIABLE EQUAL 0)
    message(FATAL_ERROR "Unable to detect Apple architecture via /usr/bin/arch")
  endif()
  if(APPLE_ARCHITECTURE STREQUAL "arm64")
    set(APPLE_ARM64 ON)
  else()
    set(APPLE_ARM64 OFF)
  endif()
else()
  set(APPLE_ARM64 OFF)
endif()

include(ExternalProject)
include(projects.cmake)
message(STATUS "zlib: ${zlib_url}")

set (COMMON_EP_ARGS
    DOWNLOAD_DIR ${CMAKE_CURRENT_BINARY_DIR}/Downloads
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}
    INSTALL_DIR ${CMAKE_INSTALL_PREFIX}
    )
set (COMMON_CMAKE_EP_ARGS
    CMAKE_GENERATOR ${CMAKE_GENERATOR}
    CMAKE_GENERATOR_PLATFORM ${CMAKE_GENERATOR_PLATFORM}
    CMAKE_GENERATOR_TOOLSET ${CMAKE_GENERATOR_TOOLSET}
    )

if(CMAKE_GENERATOR STREQUAL Ninja)
  list(APPEND COMMON_CMAKE_EP_ARGS USES_TERMINAL_BUILD 1)
endif()
set (COMMON_CMAKE_ARGS
    -DCMAKE_BUILD_TYPE=Release
    -DBUILD_SHARED_LIBS:BOOL=OFF
    -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=ON
    -DCMAKE_C_FLAGS=${CMAKE_C_FLAGS}
    -DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}
    -DCMAKE_CXX_STANDARD:STRING=17
    -DCMAKE_PREFIX_PATH:PATH=${CMAKE_INSTALL_PREFIX}
    -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
    )
if(APPLE)
  if(APPLE_ARM64)
    list(APPEND COMMON_CMAKE_ARGS "-DCMAKE_OSX_ARCHITECTURES=arm64")
  else()
    list(APPEND COMMON_CMAKE_ARGS "-DCMAKE_OSX_ARCHITECTURES=x86_64")
  endif()
endif()

function(extract_license PROJECT)
    set(command "")
    foreach(file IN LISTS ARGN)
        list(APPEND command
            COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_BINARY_DIR}/src/${PROJECT}/${file}
            ${CMAKE_INSTALL_PREFIX}/licenses/${PROJECT}/${file}
        )
    endforeach()
    ExternalProject_Add_Step(
        ${PROJECT} CopyLicense
        ${command}
        DEPENDEES install
    )
endfunction()

set_property(DIRECTORY PROPERTY EP_STEP_TARGETS download)
foreach(project ${ALL_PROJECTS})
    include(projects/${project}.cmake)
endforeach()
