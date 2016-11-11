option(FIND_PACKAGE_STRICT
  "Stop processing with an error message if WITH_<PACKAGE> is enabled, but <PACKAGE> is not found"
  ON)
mark_as_advanced(FIND_PACKAGE_STRICT)

#------------------------------------------------------------------------------
# Find and load settings from an external project. <PACKAGE>_FOUND will be set
# to indicate whether the package was found.
#
#   drake_find_package(<PACKAGE> [REQUIRED] [QUIET | NO_QUIET]
#                      [CONFIG | MODULE | PKG_CONFIG] [<ARGS>])
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
#     Use find_package in config mode to find the package.
#   MODULE
#     Use find_package in module mode to find the package.
#   PKG_CONFIG
#     Use pkg-config to find the package.
#   <ARGS>
#    Additional arguments to be passed through to find_package or
#    pkg_check_modules.
#------------------------------------------------------------------------------
macro(drake_find_package PACKAGE)
  cmake_parse_arguments(DFP "NO_QUIET;PKG_CONFIG;QUIET;REQUIRED" "" "" ${ARGN})

  if(DFP_REQUIRED)
    set(dfp_required REQUIRED)
  endif()

  if(NOT DFP_NO_QUIET AND (DFP_QUIET OR NOT DFP_REQUIRED))
    set(dfp_quiet QUIET)
  endif()

  if(DFP_PKG_CONFIG)
    find_package(PkgConfig MODULE REQUIRED)
    set(PKG_CONFIG_USE_CMAKE_PREFIX_PATH ON)
    pkg_check_modules("${PACKAGE}" ${dfp_required} ${dfp_quiet}
      ${DFP_UNPARSED_ARGUMENTS} "${PACKAGE}")
  else()
    find_package("${PACKAGE}" ${dfp_required} ${dfp_quiet}
      ${DFP_UNPARSED_ARGUMENTS})
  endif()

  string(TOUPPER "${PACKAGE}" dfp_package_upper)

  if(NOT ${PACKAGE}_FOUND AND FIND_PACKAGE_STRICT AND WITH_${dfp_package_upper})
    message(FATAL_ERROR
      "WITH_${dfp_package_upper} is enabled, but could NOT find ${PACKAGE}")
  endif()

  unset(dfp_package_upper)
  unset(dfp_quiet)
  unset(dfp_required)

  unset(DFP_NO_QUIET)
  unset(DFP_PKG_CONFIG)
  unset(DFP_QUIET)
  unset(DFP_REQUIRED)
  unset(DFP_UNPARSED_ARGUMENTS)
endmacro()

#------------------------------------------------------------------------------
# Find external packages.
#------------------------------------------------------------------------------
macro(drake_find_packages)
  #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  # BEGIN required packages

  drake_find_package(Eigen3 CONFIG REQUIRED)
  add_definitions(-DEIGEN_MPL2_ONLY)  # Per #4065.
  drake_find_package(gflags CONFIG REQUIRED)
  drake_find_package(GTest MODULE REQUIRED)

  # END required packages
  #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  # BEGIN optional packages

  drake_find_package(avl CONFIG)
  drake_find_package(bot2-core CONFIG)
  drake_find_package(Bullet MODULE)
  drake_find_package(gurobi CONFIG)
  drake_find_package(lcm CONFIG)
  drake_find_package(meshconverters CONFIG)
  drake_find_package(mosek PKG_CONFIG)
  drake_find_package(NLopt CONFIG)
  drake_find_package(octomap CONFIG)
  drake_find_package(robotlocomotion-lcmtypes CONFIG)
  drake_find_package(snopt CONFIG)
  drake_find_package(spdlog CONFIG)
  drake_find_package(xfoil CONFIG)
  drake_find_package(yaml-cpp CONFIG)

  # END optional packages
  #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
endmacro()
