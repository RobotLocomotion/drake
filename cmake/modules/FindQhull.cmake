#.rst:
#
# FindQhull
# -----------
#
# Find the Qhull headers and libraries.
#
# Imported targets
# ^^^^^^^^^^^^^^^^
#
# This module defines the following :prop_tgt:`IMPORTED` targets:
#
# ``Qhull::qhull``
#   The Qhull C library, if found.
# ``Qhull::qhullcpp``
#   The Qhull C++ library, if found.
#
# Result variables
# ^^^^^^^^^^^^^^^^
#
# This module will set the following variables in your project:
#
# ``QHULL_FOUND``
#   TRUE if the Qhull C and C++ headers and libraries were found.
# ``QHULL_VERSION``
#   The Qhull release version.
# ``QHULL_INCLUDE_DIRS``
#   The directory containing the Qhull C headers.
# ``QHULL_LIBRARIES``
#   The Qhull C libraries to be linked.
# ``QHULLCPP_INCLUDE_DIRS``
#   The directory containing the Qhull C++ headers.
# ``QHULLCPP_LIBRARIES``
#   The Qhull C++ libraries to be linked.
#
# Cache variables
# ^^^^^^^^^^^^^^^
#
# The following cache variables may be set:
#
# ``QHULL_INCLUDE_DIR``
#   The absolute path to the directory containing the Qhull C headers.
# ``QHULL_LIBRARY``
#   The absolute path to the Qhull C library.
# ``QHULLCPP_INCLUDE_DIR``
#   The absolute path to the directory containing the Qhull C++ headers.
# ``QHULLCPP_LIBRARY``
#   The absolute path to the Qhull C++ library.
# ``QHULL_VERSION``
#   The Qhull release version.

find_package(PkgConfig QUIET)

if(PKG_CONFIG_FOUND)
  set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH ON)

  pkg_check_modules(_PC_QHULL qhull)

  set(QHULL_VERSION "${_PC_QHULL_VERSION}")
endif()

find_path(QHULL_INCLUDE_DIR
  NAMES libqhull_r/libqhull_r.h libqhull/libqhull.h qhull.h
  HINTS "${_PC_QHULL_INCLUDE_DIRS}")

find_library(QHULL_LIBRARY_DEBUG
  NAMES qhull_rd qhullstatic_rd qhull_d qhullstatic_d
  HINTS "${_PC_QHULL_LIBRARY_DIRS}")

find_library(QHULL_LIBRARY_RELEASE
  NAMES qhull_r qhullstatic_r qhull qhullstatic
  HINTS "${_PC_QHULL_LIBRARY_DIRS}")

find_path(QHULLCPP_INCLUDE_DIR
  NAMES libqhullcpp/Qhull.h
  HINTS "${QHULL_INCLUDE_DIR}" "${_PC_QHULL_INCLUDE_DIRS}")

find_library(QHULLCPP_LIBRARY_DEBUG
  NAMES qhullcpp_d
  HINTS "${_PC_QHULL_LIBRARY_DIRS}")

find_library(QHULLCPP_LIBRARY_RELEASE
  NAMES qhullcpp
  HINTS "${_PC_QHULL_LIBRARY_DIRS}")

include(SelectLibraryConfigurations)

select_library_configurations(QHULL)
select_library_configurations(QHULLCPP)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(Qhull
  FOUND_VAR QHULL_FOUND
  VERSION_VAR QHULL_VERSION
  REQUIRED_VARS
    QHULL_INCLUDE_DIR
    QHULL_LIBRARIES
    QHULLCPP_INCLUDE_DIR
    QHULLCPP_LIBRARIES)

if(QHULL_FOUND)
  set(QHULL_INCLUDE_DIRS "${QHULL_INCLUDE_DIR}")
  set(QHULLCPP_INCLUDE_DIRS "${QHULLCPP_INCLUDE_DIR}")

  mark_as_advanced(QHULL_INCLUDE_DIR QHULLCPP_INCLUDE_DIR)

  if(NOT TARGET Qhull::qhull)
    add_library(Qhull::qhull UNKNOWN IMPORTED)

    set_target_properties(Qhull::qhull PROPERTIES
      IMPORTED_LINK_INTERFACE_LANGUAGES "C"
      INTERFACE_INCLUDE_DIRECTORIES "${QHULL_INCLUDE_DIR}")

    if(QHULL_LIBRARY_RELEASE)
      set_property(TARGET Qhull::qhull APPEND PROPERTY
        IMPORTED_CONFIGURATIONS RELEASE)
      set_target_properties(Qhull::qhull PROPERTIES
        IMPORTED_LOCATION_RELEASE "${QHULL_LIBRARY_RELEASE}")
    endif()

    if(QHULL_LIBRARY_DEBUG)
      set_property(TARGET Qhull::qhull APPEND PROPERTY
        IMPORTED_CONFIGURATIONS DEBUG)
      set_target_properties(Qhull::qhull PROPERTIES
        IMPORTED_LOCATION_DEBUG "${QHULL_LIBRARY_DEBUG}")
    endif()

    if(NOT QHULL_LIBRARY_RELEASE AND NOT QHULL_LIBRARY_DEBUG)
      set_property(TARGET Qhull::qhull APPEND PROPERTY
        IMPORTED_LOCATION "${QHULL_LIBRARY}")
    endif()
  endif()

  if(NOT TARGET Qhull::qhullcpp)
    add_library(Qhull::qhullcpp UNKNOWN IMPORTED)

    set_target_properties(Qhull::qhullcpp PROPERTIES
      IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
      INTERFACE_INCLUDE_DIRECTORIES "${QHULLCPP_INCLUDE_DIR}"
      INTERFACE_LINK_LIBRARIES Qhull::qhull)

    if(QHULLCPP_LIBRARY_RELEASE)
      set_property(TARGET Qhull::qhullcpp APPEND PROPERTY
        IMPORTED_CONFIGURATIONS RELEASE)
      set_target_properties(Qhull::qhullcpp PROPERTIES
        IMPORTED_LOCATION_RELEASE "${QHULLCPP_LIBRARY_RELEASE}")
    endif()

    if(QHULLCPP_LIBRARY_DEBUG)
      set_property(TARGET Qhull::qhullcpp APPEND PROPERTY
        IMPORTED_CONFIGURATIONS DEBUG)
      set_target_properties(Qhull::qhullcpp PROPERTIES
        IMPORTED_LOCATION_DEBUG "${QHULLCPP_LIBRARY_DEBUG}")
    endif()

    if(NOT QHULLCPP_LIBRARY_RELEASE AND NOT QHULLCPP_LIBRARY_DEBUG)
      set_property(TARGET Qhull::qhullcpp APPEND PROPERTY
        IMPORTED_LOCATION "${QHULLCPP_LIBRARY}")
    endif()
  endif()
endif()
