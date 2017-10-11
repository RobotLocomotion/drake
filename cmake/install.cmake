#------------------------------------------------------------------------------
# Install a list of header files to the "include" installation directory.
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

  file(RELATIVE_PATH _RELATIVE_PATH
    "${PROJECT_SOURCE_DIR}/"
    "${CMAKE_CURRENT_SOURCE_DIR}")

  install(FILES ${_FILES}
    DESTINATION "include/drake/${_RELATIVE_PATH}"
    ${_UNPARSED_ARGUMENTS})
endfunction()

#------------------------------------------------------------------------------
# Install a list of library targets to the "lib" installation directory.
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
#------------------------------------------------------------------------------
function(drake_install_libraries)
  cmake_parse_arguments("" "" "" "TARGETS" ${ARGN})

  if(NOT _TARGETS)
    set(_TARGETS ${_UNPARSED_ARGUMENTS})
    set(_UNPARSED_ARGUMENTS)
  endif()

  install(TARGETS ${_TARGETS}
    DESTINATION lib
    ${_UNPARSED_ARGUMENTS})
endfunction()
