# -*- mode: cmake -*-
# vi: set ft=cmake :

find_path(Gurobi_INCLUDE_DIR NAMES gurobi_c.h
  PATHS
    /Library/gurobi902/mac64
    /opt/gurobi902/linux64
    /opt/gurobi902/power64
    ENV GUROBI_HOME
  PATH_SUFFIXES include
)

file(STRINGS "${Gurobi_INCLUDE_DIR}/gurobi_c.h" _GUROBI_VERSION_STRINGS
  REGEX "^#define GRB_VERSION_(MAJOR|MINOR|TECHNICAL)[ ]+[0-9]+$"
)

string(REGEX REPLACE ".*;#define GRB_VERSION_MAJOR[ ]+([0-9]+);.*"
  "\\1" Gurobi_VERSION_MAJOR ";${_GUROBI_VERSION_STRINGS};"
)

string(REGEX REPLACE ".*;#define GRB_VERSION_MINOR[ ]+([0-9]+);.*"
  "\\1" Gurobi_VERSION_MINOR ";${_GUROBI_VERSION_STRINGS};"
)

string(REGEX REPLACE ".*;#define GRB_VERSION_TECHNICAL[ ]+([0-9]+);.*"
  "\\1" Gurobi_VERSION_PATCH ";${_GUROBI_VERSION_STRINGS};"
)

unset(_GUROBI_VERSION_STRINGS)

set(Gurobi_VERSION
  "${Gurobi_VERSION_MAJOR}.${Gurobi_VERSION_MINOR}.${Gurobi_VERSION_PATCH}"
)

get_filename_component(_GUROBI_ROOT "${Gurobi_INCLUDE_DIR}" DIRECTORY)

find_library(Gurobi_LIBRARY
  NAMES "gurobi${Gurobi_VERSION_MAJOR}${Gurobi_VERSION_MINOR}"
  HINTS "${_GUROBI_ROOT}"
  PATHS
    /Library/gurobi902/mac64
    /opt/gurobi902/linux64
    /opt/gurobi902/power64
    ENV GUROBI_HOME
  PATH_SUFFIXES lib
)

unset(_GUROBI_ROOT)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(Gurobi
  FOUND_VAR Gurobi_FOUND
  REQUIRED_VARS Gurobi_INCLUDE_DIR Gurobi_LIBRARY
  VERSION_VAR Gurobi_VERSION
)

if(Gurobi_FOUND)
  set(Gurobi_INCLUDE_DIRS "${Gurobi_INCLUDE_DIR}")
  set(Gurobi_LIBRARIES "${Gurobi_LIBRARY}")

  mark_as_advanced(Gurobi_INCLUDE_DIR Gurobi_LIBRARY)
endif()
