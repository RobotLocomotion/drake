include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

set(DRAKE_INSTALL_INCLUDE_DIR "${CMAKE_INSTALL_INCLUDEDIR}")
set(DRAKE_INSTALL_LIBRARY_DIR "${CMAKE_INSTALL_LIBDIR}")
set(DRAKE_INSTALL_RUNTIME_DIR "${CMAKE_INSTALL_BINDIR}")
set(DRAKE_INSTALL_DOCUMENTATION_DIR "${CMAKE_INSTALL_DOCDIR}")

set(DRAKE_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/${DRAKE_INSTALL_INCLUDE_DIR}")
set(DRAKE_LIBRARY_DIR "${CMAKE_INSTALL_PREFIX}/${DRAKE_INSTALL_LIBRARY_DIR}")
set(DRAKE_RUNTIME_DIR "${CMAKE_INSTALL_PREFIX}/${DRAKE_INSTALL_RUNTIME_DIR}")
set(DRAKE_DOCUMENTATION_DIR "${CMAKE_INSTALL_PREFIX}/${DRAKE_INSTALL_DOCUMENTATION_DIR}")

#------------------------------------------------------------------------------
# Install a list of header files to the "include" installation directory of the
# drake project.
#
#   drake_install_headers(<files...>)
#
#   drake_install_headers([<args...>] FILES <files...>)
#
# Arguments:
#   FILES <files...>
#     List of header file names. File names given as relative paths are
#     interpreted with respect to the current source directory. Files installed
#     by this function are given permissions OWNER_READ, OWNER_WRITE, GROUP_READ,
#     and WORLD_READ.
#
#   <args...>
#     Additional arguments to be passed through to install(FILES).
#------------------------------------------------------------------------------
function(drake_install_headers)
  cmake_parse_arguments("" "" "" "FILES" ${ARGN})

  if(NOT _FILES)
    set(_FILES ${_UNPARSED_ARGUMENTS})
    set(_UNPARSED_ARGUMENTS)
  endif()

  file(RELATIVE_PATH _relative_path
    "${PROJECT_SOURCE_DIR}/"
    "${CMAKE_CURRENT_SOURCE_DIR}")

  install(FILES ${_FILES}
    DESTINATION "${DRAKE_INSTALL_INCLUDE_DIR}/drake/${_relative_path}"
    ${_UNPARSED_ARGUMENTS})
endfunction()

#------------------------------------------------------------------------------
# Install a list of archive or library targets to the library installation
# directory ("lib", "lib32", or "lib64") of the drake project. Add the library
# targets to "drake-targets.cmake".
#
# drake_install_libraries(<targets...>)
#
# drake_install_libraries([<args...>] TARGETS <targets...>)
#
# Arguments:
#   TARGETS <targets...>
#     List of library targets to install.
#
#   <args...>
#     Additional arguments to be passed through to install(TARGETS).
#
# See the documentation of the "GNUInstallDirs" module for the rules that are
# used to choose the library installation directory for a given host operating
# system.
#------------------------------------------------------------------------------
function(drake_install_libraries)
  cmake_parse_arguments("" "" "" "TARGETS" ${ARGN})

  if(NOT _TARGETS)
    set(_TARGETS ${_UNPARSED_ARGUMENTS})
    set(_UNPARSED_ARGUMENTS)
  endif()

  install(TARGETS ${_TARGETS}
    EXPORT ${PROJECT_NAME}-targets
    ARCHIVE DESTINATION "${DRAKE_INSTALL_LIBRARY_DIR}"
    LIBRARY DESTINATION "${DRAKE_INSTALL_LIBRARY_DIR}"
    RUNTIME DESTINATION "${DRAKE_INSTALL_RUNTIME_DIR}"
    INCLUDES DESTINATION "${DRAKE_INSTALL_INCLUDE_DIR}"
    ${_UNPARSED_ARGUMENTS})
endfunction()
