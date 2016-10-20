option(FIND_PACKAGE_STRICT
  "Stop processing with an error message if WITH_<PACKAGE> is enabled, but <PACKAGE> is not found"
  ON)
mark_as_advanced(FIND_PACKAGE_STRICT)

#------------------------------------------------------------------------------
# Find and load settings from an external project. <PACKAGE>_FOUND will be set
# to indicate whether the package was found.
#
#   drake_find_package(<PACKAGE> [REQUIRED] [QUIET | NO_QUIET]
#                      [CONFIG | MODULE])
#
# Arguments:
#   <PACKAGE>
#     The name of the package to find.
#   REQUIRED
#     Stop processing with an error message if the package cannot be found
#   QUIET
#     Disable warning messages if the package cannot be found. The default for
#     packages that are not REQUIRED.
#   NO_QUIET
#     Enable messages if the package cannot be found. The default for packages
#     that are REQUIRED.
#   CONFIG
#     Use config mode to find the package.
#   MODULE
#     Use module mode to find the package.
#------------------------------------------------------------------------------
macro(drake_find_package PACKAGE)
  cmake_parse_arguments(DFP "NO_QUIET;QUIET;REQUIRED" "" "" ${ARGN})

  if(DFP_REQUIRED)
    set(dfp_required REQUIRED)
  endif()

  if(NOT DFP_NO_QUIET AND (DFP_QUIET OR NOT DFP_REQUIRED))
    set(dfp_quiet QUIET)
  endif()

  find_package("${PACKAGE}" ${dfp_required} ${dfp_quiet}
    ${DFP_UNPARSED_ARGUMENTS})

  string(TOUPPER "${PACKAGE}" dfp_package_upper)

  if(NOT ${PACKAGE}_FOUND AND FIND_PACKAGE_STRICT AND WITH_${dfp_package_upper})
    message(FATAL_ERROR
      "WITH_${dfp_package_upper} is enabled, but could NOT find ${PACKAGE}")
  endif()

  unset(dfp_package_upper)
  unset(dfp_quiet)
  unset(dfp_required)

  unset(DFP_NO_QUIET)
  unset(DFP_QUIET)
  unset(DFP_REQUIRED)
  unset(DFP_UNPARSED_ARGUMENTS)
endmacro()
