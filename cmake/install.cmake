include(GNUInstallDirs)

set(DRAKE_INSTALL_INCLUDE_DIR "${CMAKE_INSTALL_INCLUDEDIR}")
set(DRAKE_INSTALL_LIBRARY_DIR "${CMAKE_INSTALL_LIBDIR}")
set(DRAKE_INSTALL_PKGCONFIG_DIR "${CMAKE_INSTALL_LIBDIR}/pkgconfig")
set(DRAKE_INSTALL_RUNTIME_DIR "${CMAKE_INSTALL_BINDIR}")

set(DRAKE_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/${DRAKE_INSTALL_INCLUDE_DIR}")
set(DRAKE_LIBRARY_DIR "${CMAKE_INSTALL_PREFIX}/${DRAKE_INSTALL_LIBRARY_DIR}")
set(DRAKE_PKGCONFIG_DIR "${CMAKE_INSTALL_PREFIX}/${DRAKE_INSTALL_PKGCONFIG_DIR}")
set(DRAKE_RUNTIME_DIR "${CMAKE_INSTALL_PREFIX}/${DRAKE_INSTALL_RUNTIME_DIR}")

#------------------------------------------------------------------------------
# Install a list of runtime targets to the "bin" installation directory of the
# drake project.
#
#   drake_install_executables(<targets...>)
#
#   drake_install_executables([<args...>] TARGETS <targets...>)
#
# Arguments:
#   TARGETS <targets...>
#     List of runtime targets. Files installed by this function are given
#     permissions OWNER_EXECUTE, OWNER_READ, OWNER_WRITE, GROUP_READ,
#     GROUP_EXECUTE, WORLD_READ, and WORLD_EXECUTE.
#
#   <args...>
#     Additional arguments to be passed through to install(TARGETS).
#------------------------------------------------------------------------------
function(drake_install_executables)
  cmake_parse_arguments("" "" "" "TARGETS" ${ARGN})

  if(NOT _TARGETS)
    set(_TARGETS ${_UNPARSED_ARGUMENTS})
    set(_UNPARSED_ARGUMENTS)
  endif()

  install(TARGETS ${_TARGETS}
    RUNTIME DESTINATION "${DRAKE_INSTALL_RUNTIME_DIR}"
    ${_UNPARSED_ARGUMENTS})
endfunction()

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
    DESTINATION "${DRAKE_INSTALL_INCLUDE_DIR}/drake/${_relative_path}")
endfunction()

#------------------------------------------------------------------------------
# Install a list of archive or library targets to the library installation
# directory ("lib", "lib32", or "lib64") of the drake project.
#
# drake_install_libraries(<targets...>)
#
# drake_install_libraries([<args...>] TARGETS <targets...>)
#
# Arguments:
#   TARGETS <targets...>
#     List of library targets.
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
    ARCHIVE DESTINATION "${DRAKE_INSTALL_LIBRARY_DIR}"
    LIBRARY DESTINATION "${DRAKE_INSTALL_LIBRARY_DIR}"
    RUNTIME DESTINATION "${DRAKE_INSTALL_RUNTIME_DIR}"
    ${_UNPARSED_ARGUMENTS})
endfunction()

#------------------------------------------------------------------------------
# Generate and install a pkg-config .pc file for a package to the "pkgconfig"
# subdirectory of the library installation directory ("lib", "lib32", or
# "lib64") of the drake project.
#
# drake_install_pkg_config_file(<name> [TARGET <target>]
#                               [DESCRIPTION <description>] [URL <url>]
#                               [VERSION <major.minor.patch>]
#                               [REQUIRES <packages...>]
#                               [LIBS <flags...>] [CFLAGS <flags...>]
#                               [CLASSPATH <paths...>])
#
# Arguments:
#   <name>
#     Name of the pkg-config package. The generated pkg-config file will be
#     named "<name>.pc".
#
#   TARGET <target>
#     Target associated with the pkg-config package. Reserved for future use.
#
#   DESCRIPTION <description>
#     Description field of the package.
#
#   URL <url>
#     URL field of the package or "http://drake.mit.edu/" if not specified.
#
#   VERSION <major.minor.patch>
#     Version field of the package or PROJECT_VERSION if not specified.
#
#   REQUIRES <packages...>
#     Requires field of the package.
#
#   LIBS <flags...>
#     Libs field of the package. The flag "-L${libdir}" is automatically
#     prepended to the field.
#
#   CFLAGS <flags...>
#     Cflags field of the package. The flag "-I${includedir}" automatically
#     prepended to the field.
#
#   CLASSPATH <paths...>
#     Non-standard classpath field of the package.
#
# See the documentation of the "GNUInstallDirs" module for the rules that are
# used to choose the library installation directory for a given host operating
# system.
#------------------------------------------------------------------------------
function(drake_install_pkg_config_file _NAME)
  cmake_parse_arguments("" "" "DESCRIPTION;TARGET;URL;VERSION"
    "CFLAGS;CLASSPATH;LIBS;REQUIRES" ${ARGN})

  if(_TARGET)
    # TODO(jamiesnape): Generate _REQUIRES automatically using this property
    set_target_properties(${_TARGET} PROPERTIES PKG_CONFIG_NAME ${_NAME})
  endif()

  if(NOT _URL)
    set(_URL "http://drake.mit.edu/")
  endif()

  if(NOT _VERSION)
    set(_VERSION "${PROJECT_VERSION}")
  endif()

  if(_CLASSPATH)
    string(REPLACE ";" ":" _classpath "${_CLASSPATH}")
    set(_classpath "classpath=${_classpath}")
  else()
    set(_classpath)
  endif()

  if(_LIBS)
    string(REPLACE ";" " " _libs "${_LIBS}")
    set(_libs "-L\${libdir} ${_libs}")
  else()
    set(_libs)
  endif()

  string(REPLACE ";" " " _cflags "${_CFLAGS}")
  string(REPLACE ";" " " _requires "${_REQUIRES}")

  set(_pkg_config_file "${CMAKE_CURRENT_BINARY_DIR}/${_NAME}.pc")

  configure_file("${PROJECT_SOURCE_DIR}/../cmake/template.pc.in"
    "${_pkg_config_file}" @ONLY)

  install(FILES "${_pkg_config_file}"
    DESTINATION "${DRAKE_INSTALL_PKGCONFIG_DIR}")
endfunction()
