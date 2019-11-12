# -*- mode: cmake -*-
# vi: set ft=cmake :

find_package(PkgConfig QUIET)

pkg_check_modules(_PC_TinyXML2 QUIET tinyxml2)

find_path(TinyXML2_INCLUDE_DIR
  NAMES tinyxml2.h
  PATHS ${_PC_TinyXML2_INCLUDE_DIRS}
  PATH_SUFFIXES TinyXML2
)

find_library(TinyXML2_LIBRARY
  NAMES tinyxml2
  PATHS ${_PC_TinyXML2_LIBRARY_DIRS}
)

if(_PC_TinyXML2_VERSION)
  set(TinyXML2_VERSION "${_PC_TinyXML2_VERSION}")
elseif(TinyXML2_INCLUDE_DIR)
  file(STRINGS "${TinyXML2_INCLUDE_DIR}/tinyxml2.h" _TinyXML2_VERSION_STRINGS
    REGEX "^#define TINYXML2_(MAJOR|MINOR|PATCH)_VERSION[ ]+[0-9]+$"
  )

  string(REGEX REPLACE ".*;#define TINYXML2_MAJOR_VERSION[ ]+([0-9]+);.*"
    "\\1" TinyXML2_VERSION_MAJOR ";${_TinyXML2_VERSION_STRINGS};"
  )

  string(REGEX REPLACE ".*;#define TINYXML2_MINOR_VERSION[ ]+([0-9]+);.*"
    "\\1" TinyXML2_VERSION_MINOR ";${_TinyXML2_VERSION_STRINGS};"
  )

  string(REGEX REPLACE ".*;#define TINYXML2_PATCH_VERSION[ ]+([0-9]+);.*"
    "\\1" TinyXML2_VERSION_PATCH ";${_TinyXML2_VERSION_STRINGS};"
  )

  set(TinyXML2_VERSION
    "${TinyXML2_VERSION_MAJOR}.${TinyXML2_VERSION_MINOR}.${TinyXML2_VERSION_PATCH}"
  )

  unset(_TinyXML2_VERSION_STRINGS)
else()
  set(TinyXML2_VERSION)
endif()

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(TinyXML2
  REQUIRED_VARS TinyXML2_INCLUDE_DIR TinyXML2_LIBRARY
  VERSION_VAR TinyXML2_VERSION
)

if(TinyXML2_FOUND)
  set(TinyXML2_INCLUDE_DIRS "${TinyXML2_INCLUDE_DIR}")
  set(TinyXML2_LIBRARIES "${TinyXML2_LIBRARY}")

  mark_as_advanced(TinyXML2_INCLUDE_DIR TinyXML2_LIBRARY)

  if(NOT TARGET tinyxml2::tinyxml2)
    add_library(tinyxml2::tinyxml2 UNKNOWN IMPORTED)
    set_target_properties(tinyxml2::tinyxml2 PROPERTIES
      IMPORTED_LOCATION "${TinyXML2_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${TinyXML2_INCLUDE_DIR}"
    )
  endif()
endif()
