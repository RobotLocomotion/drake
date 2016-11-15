include(GNUInstallDirs)

set(DRAKE_INSTALL_INCLUDE_DIR "${CMAKE_INSTALL_INCLUDEDIR}")
set(DRAKE_INSTALL_LIBRARY_DIR "${CMAKE_INSTALL_LIBDIR}")
set(DRAKE_INSTALL_PKGCONFIG_DIR "${CMAKE_INSTALL_LIBDIR}/pkgconfig")

set(DRAKE_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/${DRAKE_INSTALL_INCLUDE_DIR}")
set(DRAKE_LIBRARY_DIR "${CMAKE_INSTALL_PREFIX}/${DRAKE_INSTALL_LIBRARY_DIR}")
set(DRAKE_PKGCONFIG_DIR "${CMAKE_INSTALL_PREFIX}/${DRAKE_INSTALL_PKGCONFIG_DIR}")


#------------------------------------------------------------------------------
# Generate and install a pkg-config .pc file for a package to the "pkgconfig"
# subdirectory of the library installation directory ("lib", "lib32", or
# "lib64") of the drake project.
#
# drake_install_pkg_config_file(<NAME> [DESCRIPTION <description>] [URL <url>]
#                               [VERSION <major.minor.patch>]
#                               [REQUIRES <packages>] [LIBS <flags>]
#                               [CFLAGS <flags>] [CLASSPATH <classpath>])
#
# Arguments:
#   <NAME>
#     Name of the pkg-config package. The generated pkg-config file will be
#     named "<NAME>.pc".
#   DESCRIPTION
#     Description field of the package.
#   URL
#     URL field of the package or "http://drake.mit.edu/" if not specified.
#   VERSION
#     Version field of the package or PROJECT_VERSIION if not specified.
#   REQUIRES
#     Requires field of the package.
#   LIBS
#     Libs field of the package. The flag "-L${libdir}" is automatically
#     prepended to the field.
#   CFLAGS
#     Cflags field of the package. The flag "-I${includedir}" automatically
#     prepended to the field.
#   CLASSPATH
#     Non-standard classpath field of the package.
#
# See the documentation of the "GNUInstallDirs" module for the rules that are
# used to choose the library installation directory for a given host operating
# system.
#------------------------------------------------------------------------------
function(drake_install_pkg_config_file _NAME)
  cmake_parse_arguments("" "" "DESCRIPTION;URL;VERSION"
    "CFLAGS;CLASSPATH;LIBS;REQUIRES" ${ARGN})

  if(NOT _URL)
    set(_URL "http://drake.mit.edu/")
  endif()

  if(NOT _VERSION)
    set(_VERSION "${PROJECT_VERSION}")
  endif()

  if(_CLASSPATH)
    string(REPLACE ";" " " _classpath "${_CLASSPATH}")
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
