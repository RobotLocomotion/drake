# -*- mode: cmake -*-
# vi: set ft=cmake :

function(_gflags_find_library _NAME)
  string(TOUPPER ${_NAME} _NAME_UPPER)

  set(_LIBRARY_VAR GFLAGS_${_NAME_UPPER}_LIBRARY)
  set(_TARGET gflags::gflags_${_NAME})

  if(_NAME MATCHES static$)
    set(_SHARED_STATIC STATIC)
  else()
    set(_SHARED_STATIC SHARED)
  endif()

  if(_NAME MATCHES ^nothreads)
    set(_NOTHREADS _nothreads)
  else()
    set(_NOTHREADS)
  endif()

  find_library(${_LIBRARY_VAR}
    NAMES ${CMAKE_${_SHARED_STATIC}_LIBRARY_PREFIX}gflags${_NOTHREADS}${CMAKE_${_SHARED_STATIC}_LIBRARY_SUFFIX}
    HINTS "${PC_GFLAGS_LIBRARY_DIRS}"
  )

  set(GFlags_${_NAME}_FIND_QUIETLY ON)
  find_package_handle_standard_args(GFlags_${_NAME}
    REQUIRED_VARS GFLAGS_INCLUDE_DIR ${_LIBRARY_VAR}
  )

  if(GFlags_${_NAME}_FOUND)
    set(GFlags_${_NAME}_FOUND ON PARENT_SCOPE)
    mark_as_advanced(${_LIBRARY_VAR})

    add_library(${_TARGET} ${_SHARED_STATIC} IMPORTED)
    set_target_properties(${_TARGET} PROPERTIES
      IMPORTED_LOCATION "${${_LIBRARY_VAR}}"
      INTERFACE_INCLUDE_DIRECTORIES "${GFLAGS_INCLUDE_DIRS}"
    )

    if(_NAME MATCHES static$)
      set_target_properties(${_TARGET} PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES CXX
        INTERFACE_COMPILE_DEFINITIONS GFLAGS_IS_A_DLL=0
      )
    else()
      if(WIN32)
        set_target_properties(${_TARGET} PROPERTIES
          INTERFACE_COMPILE_DEFINITIONS GFLAGS_IS_A_DLL=1
        )
      else()
        set_target_properties(${_TARGET} PROPERTIES
          INTERFACE_COMPILE_DEFINITIONS GFLAGS_IS_A_DLL=0
        )
      endif()
    endif()

    if(_NAME STREQUAL static)
      find_package(Threads QUIET)

      if(Threads_FOUND)
        set_target_properties(${_TARGET} PROPERTIES
          INTERFACE_LINK_LIBRARIES $<LINK_ONLY:Threads::Threads>
        )
      endif()
    endif()
  endif()
endfunction()

if(NOT GFlags_FIND_COMPONENTS)
  set(GFlags_FIND_COMPONENTS nothreads_shared)
  set(GFlags_FIND_REQUIRED_nothreads_shared ${GFlags_FIND_REQUIRED})
endif()

find_package(PkgConfig QUIET)

set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH ON)
pkg_check_modules(PC_GFLAGS QUIET gflags)

if(PC_GFLAGS_VERSION)
  set(GFLAGS_VERSION "${PC_GFLAGS_VERSION}")
  set(GFLAGS_VERSION_STRING "${GFLAGS_VERSION}")
else()
  set(GFLAGS_VERSION)
  set(GFLAGS_VERSION_STRING)
endif()

find_path(GFLAGS_INCLUDE_DIR NAMES gflags/gflags.h
  HINTS "${PC_GFLAGS_INCLUDE_DIRS}"
)

foreach(_GFLAGS_COMPONENT ${GFlags_FIND_COMPONENTS})
  _gflags_find_library(${_GFLAGS_COMPONENT})
endforeach()

if(GFLAGS_NOTHREADS_SHARED_LIBRARY)
  set(GFLAGS_LIBRARY gflags::gflags_nothreads_shared)
elseif(GFLAGS_NOTHREADS_STATIC_LIBRARY)
  set(GFLAGS_LIBRARY gflags::gflags_nothreads_static)
elseif(GFLAGS_SHARED_LIBRARY)
  set(GFLAGS_LIBRARY gflags::gflags_shared)
elseif(GFLAGS_STATIC_LIBRARY)
  set(GFLAGS_LIBRARY gflags::gflags_static)
else()
  set(GFLAGS_LIBRARY)
endif()

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(GFlags
  REQUIRED_VARS GFLAGS_INCLUDE_DIR GFLAGS_LIBRARY
  VERSION_VAR GFLAGS_VERSION
  HANDLE_COMPONENTS
)

if(GFLAGS_FOUND)
  set(GFLAGS_INCLUDE_DIRS "${GFLAGS_INCLUDE_DIR}")
  set(GFLAGS_LIBRARIES "${GFLAGS_LIBRARY}")
  mark_as_advanced(GFLAGS_INCLUDE_DIR)
endif()
