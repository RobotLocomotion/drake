include(CMakeDependentOption)

#------------------------------------------------------------------------------
# Add an option with optional dependencies.
#
# Arguments:
#   <DEFAULT_STATE> - Is the option enabled by default? (`ON` or `OFF`)
#
#   DEPENDS <expression>
#     A list of expressions which must evaluate to true for the component to
#     be made available. (Otherwise, the component will not be enabled, and the
#     option for the component will be hidden.)
#
# Extra arguments are combined (with a single space) to form the description of
# the option. A ';' appearing in the description text should be escaped to
# prevent it being seen as an argument separator.
#
# This creates an option <NAME>, which may be either a dependent option or an
# unconditional option.
#------------------------------------------------------------------------------
function(drake_option NAME DEFAULT_STATE)
  # "Fix" escaping so cmake_parse_arguments will split properly (note:
  # cmake_parse_arguments splits twice, so escape two more levels).
  string(REPLACE "\\;" "\\\\\\;" _args "${ARGN}")

  # Parse arguments.
  cmake_parse_arguments(_opt "" "DEPENDS" "" ${_args})

  # Join description snippets.
  foreach(_snippet IN LISTS _opt_UNPARSED_ARGUMENTS)
    set(_description "${_description} ${_snippet}")
  endforeach()
  string(STRIP "${_description}" _description)

  # Create option.
  if(DEFINED _opt_DEPENDS)
    string(REPLACE "\\;" ";" _opt_DEPENDS "${_opt_DEPENDS}")
    cmake_dependent_option(${NAME}
      "${_description}"
      ${DEFAULT_STATE}
      "${_opt_DEPENDS}" OFF)
  else()
    option(${NAME} "${_description}" ${DEFAULT_STATE})
  endif()

  # Propagate option value to caller.
  set(${NAME} "${${NAME}}" PARENT_SCOPE)
endfunction()

#------------------------------------------------------------------------------
# Add an option whether to use the system or internal version of a dependency.
#
# Arguments:
#   OPTIONAL - Dependency is not required.
#
#   REQUIRES <package>
#     Name of package (as passed to `find_package`) that must be found if the
#     system version is selected.
#
#   VERSION <version>
#     Required version of the system package (passed to `find_package`).
#
#   DEPENDS <expression>
#     A list of expressions which must evaluate to true for the component to
#     be made available. (Otherwise, the component will not be enabled, and the
#     option for the component will be hidden.)
#
# Extra arguments are combined (with a single space) to form the description of
# the option. A ';' appearing in the description text should be escaped to
# prevent it being seen as an argument separator.
#
# This creates an option USE_SYSTEM_<NAME> that selects if the system version
# of the dependency will be used. If the component is required, and the system
# version is not selected, the internal version will be enabled. Otherwise, the
# user will be given an option (WITH_<NAME>, defaulting to ON) if the component
# should be built.
#
# Internally, this sets HAVE_<NAME> to indicate if the external is available,
# and WITH_<NAME> to indicate if our internal version of the external should be
# built.
#------------------------------------------------------------------------------
function(drake_system_dependency NAME)
  # "Fix" escaping so cmake_parse_arguments will split properly (note:
  # cmake_parse_arguments splits twice, so escape two more levels).
  string(REPLACE "\\;" "\\\\\\;" _args "${ARGN}")

  # Parse arguments
  cmake_parse_arguments("_sd"
    "OPTIONAL"
    "REQUIRES;VERSION;DEPENDS"
    ""
    ${_args})

  # Check for required arguments.
  if(NOT DEFINED _sd_REQUIRES)
    message(FATAL_ERROR "drake_system_dependency: REQUIRES must be specified")
  endif()

  # Fix up arguments
  if(NOT _sd_OPTIONAL)
    set(_else "(if OFF, the internal version will be used)")
  endif()
  if(DEFINED _sd_DEPENDS)
    set(_sd_DEPENDS DEPENDS "${_sd_DEPENDS}")
  endif()

  # Create option for using system version of external.
  drake_option(USE_SYSTEM_${NAME} OFF
    "${_sd_DEPENDS}"
    "Use the system-provided"
    "${_sd_UNPARSED_ARGUMENTS}"
    "${_else}"
    )

  set(HAVE_${NAME} FALSE PARENT_SCOPE)

  # Handle optional dependencies.
  if(_sd_OPTIONAL)
    # Set option dependencies so option to build internal version is only shown
    # if system version is not selected.
    if(NOT DEFINED _sd_DEPENDS)
      set(_sd_DEPENDS DEPENDS "NOT USE_SYSTEM_${NAME}")
    else()
      set(_sd_DEPENDS "${_sd_DEPENDS}\;NOT USE_SYSTEM_${NAME}")
    endif()

    # Add option to build internal version.
    drake_option(WITH_${NAME} ON
      "${_sd_DEPENDS}"
      "${_sd_UNPARSED_ARGUMENTS}")
    set(WITH_${NAME} "${WITH_${NAME}}" PARENT_SCOPE)
    set(HAVE_${NAME} "${WITH_${NAME}}" PARENT_SCOPE)
  else()
    # Internally, externals use WITH_<NAME> to decide if an external is enabled,
    # so map the option value to that name.
    if(USE_SYSTEM_${NAME})
      set(WITH_${NAME} OFF PARENT_SCOPE)
    else()
      set(WITH_${NAME} ON PARENT_SCOPE)
    endif()
  endif()

  # If using system version, ensure it is available.
  if(USE_SYSTEM_${NAME})
    find_package(${_sd_REQUIRES} ${_sd_VERSION} REQUIRED)
    set(HAVE_${NAME} TRUE PARENT_SCOPE)
  endif()
endfunction()

#------------------------------------------------------------------------------
# Add an option whether or not to build an optional component.
#
# The arguments are the same as drake_option. The option will be named
# WITH_<NAME>. HAVE_<NAME> will reflect if the external is available.
#------------------------------------------------------------------------------
function(drake_optional_external NAME DEFAULT_STATE)
  # Re-escape ARGN so drake_option will receive ARGN split the same way.
  string(REPLACE "\\;" "\\\\;" _args "${ARGN}")

  # Create option and make corresponding variables available to caller.
  drake_option(WITH_${NAME} ${DEFAULT_STATE} ${_args})
  set(WITH_${NAME} "${WITH_${NAME}}" PARENT_SCOPE)
  set(HAVE_${NAME} "${WITH_${NAME}}" PARENT_SCOPE)
endfunction()

#------------------------------------------------------------------------------
# Set up options
#------------------------------------------------------------------------------
macro(drake_setup_options)
  #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  # BEGIN "system" dependencies

  # These are packages that we can build, but which we allow the user to use
  # their own copy (usually provided by the system) if preferred. Some of these
  # may be required.

  drake_system_dependency(
    EIGEN REQUIRES Eigen3 VERSION 3.2.92
    "Eigen C++ matrix library")

  drake_system_dependency(
    GOOGLETEST REQUIRES GTest
    "Google testing framework")

  drake_system_dependency(
    GFLAGS REQUIRES gflags
    "Google command-line flags processing library")

  drake_system_dependency(
    LCM OPTIONAL REQUIRES lcm
    "Lightweight Communications and Marshaling IPC suite")

  drake_system_dependency(
    BOT_CORE_LCMTYPES OPTIONAL REQUIRES bot2-core
    DEPENDS "HAVE_LCM"
    "libbot2 robotics suite LCM types")

  drake_system_dependency(
    ROBOTLOCOMOTION_LCMTYPES OPTIONAL REQUIRES robotlocomotion-lcmtypes
    DEPENDS "HAVE_BOT_CORE_LCMTYPES"
    "robotlocomotion LCM types")

  drake_system_dependency(YAML_CPP OPTIONAL REQUIRES yaml-cpp
    "C++ library for reading and writing YAML configuration files")

  # END "system" dependencies
  #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  # BEGIN external projects that are ON by default

  drake_optional_external(GOOGLE_STYLEGUIDE ON
    "Google code style tools for cpplint.py style checking" ON)

  drake_optional_external(SPDLOG ON
    "Fast C++ text logging facility\; disabling will turn off text logging")

  drake_optional_external(SWIGMAKE ON
    "Helper tools to build Python & MATLAB wrappers"
    "for C++ libraries with Eigen")

  drake_optional_external(BULLET ON "Bullet library for collision detection")

  drake_optional_external(CCD ON "Convex shape Collision Detection library")

  if(NOT WIN32)
    # Not win32 yet; builds, but requires manual installation of VTKk, etc.
    drake_optional_external(DIRECTOR ON
      DEPENDS "HAVE_LCM\;HAVE_BOT_CORE_LCMTYPES"
      "VTK-based visualization tool and robot user interface")

    # Probably not on Windows until lcmgl is split out
    drake_optional_external(LIBBOT ON
      "libbot2 robotics suite\;"
      "used for its simple open-gl visualizer + lcmgl for director")

    drake_optional_external(NLOPT ON "Non-linear optimization solver")

    # IPOPT is currently disabled on Mac when MATLAB is enabled due to MATLAB
    # compatibility issues:
    # https://github.com/RobotLocomotion/drake/issues/2578
    drake_optional_external(IPOPT ON
      DEPENDS "NOT APPLE OR NOT Matlab_FOUND"
      "Interior Point Optimizer, for solving non-linear optimizations")

    drake_optional_external(OCTOMAP ON
      "3D occupancy mapping library\; provides oct-tree data structures")

    drake_optional_external(SWIG_MATLAB ON
      "A version of SWIG with MATLAB support")
  endif()

  # END external projects that are ON by default
  #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  # BEGIN external projects that are only needed when MATLAB is in use

  # The following projects are default ON when MATLAB is present and enabled.
  # Otherwise, they are hidden and default OFF.
  drake_optional_external(SEDUMI ON
    DEPENDS "NOT DISABLE_MATLAB\;Matlab_FOUND"
    "semi-definite programming solver")

  drake_optional_external(SPOTLESS ON
    DEPENDS "NOT DISABLE_MATLAB\;Matlab_FOUND"
    "polynomial optimization front-end for MATLAB")

  # The following projects are default OFF when MATLAB is present and enabled.
  # Otherwise, they are hidden and default OFF. Some of them may also be hidden
  # on Windows regardless of the status of MATLAB.
  drake_optional_external(IRIS OFF
    DEPENDS "NOT DISABLE_MATLAB\;Matlab_FOUND\;WITH_MOSEK"
    "fast approximate convex segmentation")

  drake_optional_external(YALMIP OFF
    DEPENDS "NOT DISABLE_MATLAB\;Matlab_FOUND"
    "free optimization front-end for MATLAB")

  # END external projects that are only needed when MATLAB is in use
  #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  # BEGIN external projects that are OFF by default

  drake_optional_external(SNOPT OFF
    "Sparse Non-linear Optimizer\;"
    "requires access to RobotLocomotion/snopt-pod")

  drake_optional_external(SIGNALSCOPE OFF DEPENDS "NOT WIN32\;WITH_DIRECTOR"
    "Live plotting tool for LCM messages")

  # Many of these might work on win32 with little or no work... they just
  # haven't been tried.
  if(NOT WIN32)
    drake_optional_external(AVL OFF
      "use w/ AVL to compute aerodynamic coefficients for airfoils")

    drake_optional_external(DREAL OFF
      "Non-linear SMT solver and automated reasoning tool")

    drake_optional_external(GUROBI OFF
      "Convex/integer optimization solver\;"
      "free for academics (will prompt you for login bits)")

    drake_optional_external(MESHCONVERTERS OFF
      "uses vcglib to convert a few standard filetypes")

    drake_optional_external(MOSEK OFF
      "Convex optimization solver\; free for academics")

    # Almost works on windows;  the update step call to git was complaining
    # about local modifications on drake003
    drake_optional_external(TEXTBOOK OFF
      "The Underactuated Robotics textbook and its examples")

    drake_optional_external(XFOIL OFF
      "use w/ XFOIL to compute aerodynamic coefficients for airfoils")
  endif()

  # END external projects that are OFF by default
  #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
endmacro()
