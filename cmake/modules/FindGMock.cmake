# CMake - Cross Platform Makefile Generator
# Copyright 2000-2017 Kitware, Inc. and Contributors
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the distribution.
#
# * Neither the name of Kitware, Inc. nor the names of Contributors
#   may be used to endorse or promote products derived from this
#   software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# FindGMock
# ---------
#
# Locate the Google C++ Mocking Framework.
#
# Imported targets
# ^^^^^^^^^^^^^^^^
#
# This module defines the following :prop_tgt:`IMPORTED` targets:
#
# ``GMock::GMock``
#   The Google Mock ``gmock`` library, if found; adds Thread::Thread
#   automatically
# ``GMock::Main``
#   The Google Mock ``gmock_main`` library, if found
#
#
# Result variables
# ^^^^^^^^^^^^^^^^
#
# This module will set the following variables in your project:
#
# ``GMOCK_FOUND``
#   Found the Google C++ Mocking framework
# ``GMOCK_INCLUDE_DIRS``
#   the directory containing the Google Mock headers
#
# The library variables below are set as normal variables.  These
# contain debug/optimized keywords when a debugging library is found.
#
# ``GMOCK_LIBRARIES``
#   The Google Mock ``gmock`` library; note it also requires linking
#   with an appropriate thread library
# ``GMOCK_MAIN_LIBRARIES``
#   The Google Mock ``gmock_main`` library
# ``GMOCK_BOTH_LIBRARIES``
#   Both ``gmock`` and ``gmock_main``
#
# Cache variables
# ^^^^^^^^^^^^^^^
#
# The following cache variables may also be set:
#
# ``GMOCK_ROOT``
#   The root directory of the Google Mock installation (may also be
#   set as an environment variable)
#
# Example usage
# ^^^^^^^^^^^^^
#
# ::
#
#   enable_testing()
#   find_package(GMock REQUIRED)
#
#   add_executable(foo foo.cc)
#   target_link_libraries(foo GMock::GMock GMock::Main)

function(_gmock_append_debugs _endvar _library)
  if(${_library} AND ${_library}_DEBUG)
    set(_output optimized ${${_library}} debug ${${_library}_DEBUG})
  else()
    set(_output ${${_library}})
  endif()
  set(${_endvar} ${_output} PARENT_SCOPE)
endfunction()

function(_gmock_find_library _name)
  find_library(${_name}
    NAMES ${ARGN}
    HINTS
      ENV GMOCK_ROOT
      ${GMOCK_ROOT}
    PATH_SUFFIXES lib
  )
  mark_as_advanced(${_name})
endfunction()

find_path(GMOCK_INCLUDE_DIR gmock/gmock.h
  HINTS
    $ENV{GMOCK_ROOT}/include
    ${GMOCK_ROOT}/include
)
mark_as_advanced(GMOCK_INCLUDE_DIR)

_gmock_find_library(GMOCK_LIBRARY gmock)
_gmock_find_library(GMOCK_LIBRARY_DEBUG gmockd)
_gmock_find_library(GMOCK_MAIN_LIBRARY gmock_main)
_gmock_find_library(GMOCK_MAIN_LIBRARY_DEBUG gmock_maind)

include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GMock DEFAULT_MSG GMOCK_LIBRARY GMOCK_INCLUDE_DIR GMOCK_MAIN_LIBRARY)

if(GMOCK_FOUND)
  set(GMOCK_INCLUDE_DIRS ${GMOCK_INCLUDE_DIR})
  _gmock_append_debugs(GMOCK_LIBRARIES GMOCK_LIBRARY)
  _gmock_append_debugs(GMOCK_MAIN_LIBRARIES GMOCK_MAIN_LIBRARY)
  set(GMOCK_BOTH_LIBRARIES ${GMOCK_LIBRARIES} ${GMOCK_MAIN_LIBRARIES})

  include(CMakeFindDependencyMacro)
  find_dependency(Threads)

  if(NOT TARGET GMock::GMock)
    add_library(GMock::GMock UNKNOWN IMPORTED)
    set_target_properties(GMock::GMock PROPERTIES
      INTERFACE_LINK_LIBRARIES "Threads::Threads")
    if(GMOCK_INCLUDE_DIRS)
      set_target_properties(GMock::GMock PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${GMOCK_INCLUDE_DIRS}")
    endif()
    if(EXISTS "${GMOCK_LIBRARY}")
      set_target_properties(GMock::GMock PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
        IMPORTED_LOCATION "${GMOCK_LIBRARY}")
    endif()
    if(EXISTS "${GMOCK_LIBRARY_RELEASE}")
      set_property(TARGET GMock::GMock APPEND PROPERTY
        IMPORTED_CONFIGURATIONS RELEASE)
      set_target_properties(GMock::GMock PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
        IMPORTED_LOCATION_RELEASE "${GMOCK_LIBRARY_RELEASE}")
    endif()
    if(EXISTS "${GMOCK_LIBRARY_DEBUG}")
      set_property(TARGET GMock::GMock APPEND PROPERTY
        IMPORTED_CONFIGURATIONS DEBUG)
      set_target_properties(GMock::GMock PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "CXX"
        IMPORTED_LOCATION_DEBUG "${GMOCK_LIBRARY_DEBUG}")
    endif()
    endif()
    if(NOT TARGET GMock::Main)
      add_library(GMock::Main UNKNOWN IMPORTED)
      set_target_properties(GMock::Main PROPERTIES
        INTERFACE_LINK_LIBRARIES "GMock::GMock")
      if(EXISTS "${GMOCK_MAIN_LIBRARY}")
        set_target_properties(GMock::Main PROPERTIES
          IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
          IMPORTED_LOCATION "${GMOCK_MAIN_LIBRARY}")
      endif()
      if(EXISTS "${GMOCK_MAIN_LIBRARY_RELEASE}")
      set_property(TARGET GMock::Main APPEND PROPERTY
        IMPORTED_CONFIGURATIONS RELEASE)
      set_target_properties(GMock::Main PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
        IMPORTED_LOCATION_RELEASE "${GMOCK_MAIN_LIBRARY_RELEASE}")
      endif()
      if(EXISTS "${GMOCK_MAIN_LIBRARY_DEBUG}")
      set_property(TARGET GMock::Main APPEND PROPERTY
        IMPORTED_CONFIGURATIONS DEBUG)
      set_target_properties(GMock::Main PROPERTIES
        IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "CXX"
        IMPORTED_LOCATION_DEBUG "${GMOCK_MAIN_LIBRARY_DEBUG}")
      endif()
  endif()
endif()
