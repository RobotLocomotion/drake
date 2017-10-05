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
  set(_description)
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
#   PREFER_SYSTEM_VERSION
#     Default to using the system version of the dependency rather than the
#     internal version.
#
#   REQUIRES <package>
#     Name of package (as passed to `find_package`) that must be found if the
#     system version is selected.
#
#   VERSION <version>
#     Required version of the system package (passed to `find_package`).
#
#   ADDITIONAL_VERSIONS <version>
#     Versions of the system package to be accepted in addition to the named
#     VERSION.
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
    "OPTIONAL;PREFER_SYSTEM_VERSION;--"
    "REQUIRES;VERSION;DEPENDS"
    "ADDITIONAL_VERSIONS"
    ${_args})

  # Check for required arguments.
  if(NOT DEFINED _sd_REQUIRES)
    message(FATAL_ERROR "drake_system_dependency: REQUIRES must be specified")
  endif()

  # Fix up arguments
  if(_sd_OPTIONAL)
    set(_else)
  else()
    set(_else "(if OFF, the internal version will be used)")
  endif()
  if(DEFINED _sd_DEPENDS)
    set(_sd_DEPENDS DEPENDS "${_sd_DEPENDS}")
  endif()
  if(_sd_PREFER_SYSTEM_VERSION)
    set(_default ON)
  else()
    set(_default OFF)
  endif()

  # Create option for using system version of external.
  drake_option(USE_SYSTEM_${NAME} ${_default}
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
    set(_user_dir "${${NAME}_DIR}")
    foreach(_version ${_sd_ADDITIONAL_VERSIONS})
      find_package(${_sd_REQUIRES} ${_version} QUIET)
      if(${NAME}_FOUND)
        set(HAVE_${NAME} TRUE PARENT_SCOPE)
        return()
      endif()
      set(${NAME}_DIR "${_user_dir}" CACHE PATH
        "The directory containing a CMake configuration file for ${NAME}."
        FORCE)
    endforeach()
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
# Set internal flag indicating whether or not to build an optional component.
#
# This sets the variables WITH_<NAME> and HAVE_<NAME>, indicating if the
# specified external is available and will be built. These will be true iff
# <WHEN> is also true.
#------------------------------------------------------------------------------
function(drake_dependent_external NAME WHEN)
  string(REPLACE " " ";" WHEN "${WHEN}") # Force splitting
  if(${WHEN})
    set(_value ON)
  else()
    set(_value OFF)
  endif()

  set(WITH_${NAME} "${_value}" PARENT_SCOPE)
  set(HAVE_${NAME} "${_value}" PARENT_SCOPE)
endfunction()

#------------------------------------------------------------------------------
# Set up options
#------------------------------------------------------------------------------
macro(drake_setup_options)
  # These are packages that we can build, but which we allow the user to use
  # their own copy (usually provided by the system) if preferred.

  drake_system_dependency(
    EIGEN REQUIRES Eigen3 VERSION 3.3.3
    "Eigen C++ matrix library")

  drake_system_dependency(
    FMT REQUIRES fmt VERSION 3.0.1
    "Open-source formatting library for C++")

  drake_system_dependency(
    GOOGLETEST REQUIRES GTest VERSION 1.8
    "Google testing framework")

  drake_system_dependency(
    GFLAGS REQUIRES gflags
    "Google command-line flags processing library")

  drake_system_dependency(
    NLOPT REQUIRES NLopt
    "Non-linear optimization solver")

  drake_system_dependency(
    PYBIND11 REQUIRES pybind11
    "Python/C++11 interoperability tool")

  drake_system_dependency(
    PROTOBUF REQUIRES Protobuf VERSION 3.1
    "Google protocol buffers")

  drake_system_dependency(
    TINYOBJLOADER REQUIRES tinyobjloader VERSION 1.0.6
    "Library for reading wavefront mesh files")
endmacro()
