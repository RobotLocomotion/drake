cmake_minimum_required(VERSION 3.10.0)
project(pip-drake-dependencies)

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
