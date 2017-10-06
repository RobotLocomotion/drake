include(ExternalProject)

#------------------------------------------------------------------------------
# Build a variable (OUTVAR) that can be expanded into the `CMAKE_ARGS` argument
# of a call to `ExternalProject_Add` in order to propagate cache variables from
# the parent build to the external project.
#
# The list separator in propagated variables is replaced with the specified
# SEPARATOR, so that lists may be correctly passed through. The specified
# separator must match the one given to the `LIST_SEPARATOR` argument of the
# call to `ExternalProject_Add`.
#------------------------------------------------------------------------------
function(drake_build_cache_args OUTVAR SEPARATOR)
  set(_out)
  foreach(_var ${ARGN})
    if(DEFINED ${_var})
      get_property(_type CACHE ${_var} PROPERTY TYPE)
      string(REPLACE ";" "${SEPARATOR}" _value "${${_var}}")
      list(APPEND _out -D${_var}:${_type}=${_value})
    endif()
  endforeach()
  set(${OUTVAR} ${_out} PARENT_SCOPE)
endfunction()

#------------------------------------------------------------------------------
# Internal helper to set up a CMake external project.
#------------------------------------------------------------------------------
macro(drake_add_cmake_external PROJECT)
  # Set binary directory for external CMake project
  if(NOT DEFINED _ext_BINARY_DIR)
    set(_ext_BINARY_DIR ${PROJECT_BINARY_DIR}/externals/${PROJECT})
  endif()

  # Propagate build verbosity
  set(_ext_VERBOSE)
  if(CMAKE_VERBOSE_MAKEFILE)
    set(_ext_VERBOSE -DCMAKE_VERBOSE_MAKEFILE=ON)
  endif()

  # Set arguments for cache propagation
  set(_ext_LIST_SEPARATOR "!")

  set(_ext_PROPAGATE_CACHE_VARS
    CMAKE_EXPORT_NO_PACKAGE_REGISTRY
    CMAKE_FIND_PACKAGE_NO_PACKAGE_REGISTRY
    CMAKE_FIND_PACKAGE_NO_SYSTEM_PACKAGE_REGISTRY
    CMAKE_PREFIX_PATH
    CMAKE_INSTALL_PREFIX
    CMAKE_BUILD_TYPE
    BUILD_SHARED_LIBS
    CMAKE_C_COMPILER
    CMAKE_C_FLAGS
    CMAKE_CXX_COMPILER
    CMAKE_CXX_FLAGS
    CMAKE_EXE_LINKER_FLAGS
    CMAKE_MODULE_LINKER_FLAGS
    CMAKE_SHARED_LINKER_FLAGS
    CMAKE_STATIC_LINKER_FLAGS
    CMAKE_MACOSX_RPATH
    CMAKE_INSTALL_RPATH
    CMAKE_INSTALL_RPATH_USE_LINK_PATH)

  drake_build_cache_args(_ext_PROPAGATE_CACHE ${_ext_LIST_SEPARATOR}
    ${_ext_PROPAGATE_CACHE_VARS})

  # Set up the external project build
  ExternalProject_Add(${PROJECT}
    LIST_SEPARATOR "${_ext_LIST_SEPARATOR}"
    URL ${_ext_URL}
    URL_HASH ${_ext_URL_HASH}
    TLS_VERIFY 1
    SOURCE_SUBDIR ${_ext_SOURCE_SUBDIR}
    SOURCE_DIR ${_ext_SOURCE_DIR}
    BINARY_DIR ${_ext_BINARY_DIR}
    DOWNLOAD_DIR ${_ext_DOWNLOAD_DIR}
    DOWNLOAD_COMMAND ${_ext_DOWNLOAD_COMMAND}
    UPDATE_COMMAND ${_ext_UPDATE_COMMAND}
    STEP_TARGETS configure build install
    INDEPENDENT_STEP_TARGETS update
    BUILD_ALWAYS 1
    DEPENDS ${_ext_DEPENDS}
    CMAKE_GENERATOR "${CMAKE_GENERATOR}"
    CMAKE_ARGS
      ${_ext_VERBOSE}
      ${_ext_PROPAGATE_CACHE}
      ${_ext_CMAKE_ARGS})
endmacro()

#------------------------------------------------------------------------------
# Add an external project.
#
# Arguments:
#   LOCAL     - External is local to the source tree
#   MATLAB    - External uses MATLAB
#   PYTHON    - External uses Python
#
#   DEPENDS <deps...>
#       List of other external projects on which this external depends. Used to
#       ensure correct build order. (Dependencies which are not enabled will be
#       ignored.)
#
#   SOURCE_SUBDIR <dir> - Specify SOURCE_SUBDIR for external
#   SOURCE_DIR <dir> - Override default SOURCE_DIR for external
#   BINARY_DIR <dir> - Override default BINARY_DIR for external
#
#   CMAKE_ARGS <args...>
#       Additional arguments to be passed to the CMake external's CONFIGURE
#       step.
#
# Arguments with the same name as arguments to `ExternalProject_Add` generally
# have the same function and are passed through.
#------------------------------------------------------------------------------
function(drake_add_external PROJECT)
  # Parse arguments
  set(_ext_flags
    LOCAL
  )
  set(_ext_sv_args
    SOURCE_SUBDIR
    SOURCE_DIR
    BINARY_DIR
    URL
    URL_HASH
  )
  set(_ext_mv_args
    CMAKE_ARGS
    DEPENDS
  )
  cmake_parse_arguments(_ext
    "${_ext_flags}" "${_ext_sv_args}" "${_ext_mv_args}" ${ARGN})

  # Set source directory for external project
  if(NOT DEFINED _ext_SOURCE_DIR AND NOT DEFINED _ext_URL)
    set(_ext_SOURCE_DIR ${PROJECT_SOURCE_DIR}/externals/${PROJECT})
  elseif(DEFINED _ext_URL)
    set(_ext_SOURCE_DIR ${PROJECT_BINARY_DIR}/externals/${PROJECT}-src)
  endif()
  if(NOT DEFINED _ext_SOURCE_SUBDIR)
    set(_ext_SOURCE_SUBDIR .)
  endif()
  if(DEFINED _ext_URL)
    set(_ext_DOWNLOAD_DIR ${PROJECT_BINARY_DIR}/${PROJECT}-download)
  else()
    set(_ext_DOWNLOAD_DIR ${PROJECT_SOURCE_DIR})
  endif()

  # Compute project dependencies
  if(_ext_DEPENDS)
    message(STATUS
      "Preparing to build ${PROJECT} with dependencies ${_ext_DEPENDS}")
  else()
    message(STATUS
      "Preparing to build ${PROJECT}")
  endif()

  # Local "externals" have no download or update step
  if(_ext_LOCAL)
    set(_ext_DOWNLOAD_COMMAND "")
    set(_ext_UPDATE_COMMAND "")
  endif()

  # Add the external project
  drake_add_cmake_external(${PROJECT})

  # Register project in list of external projects
  list(APPEND EXTERNAL_PROJECTS ${PROJECT})
  set(EXTERNAL_PROJECTS ${EXTERNAL_PROJECTS} PARENT_SCOPE)
endfunction()

###############################################################################

# List of all enabled external projects
set(EXTERNAL_PROJECTS)
