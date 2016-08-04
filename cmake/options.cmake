include(CMakeDependentOption)

#------------------------------------------------------------------------------
# Add an option whether to use the system or internal version of a (required)
# dependency.
#------------------------------------------------------------------------------
function(drake_system_dependency NAME DESCRIPTION)
  # Backwards compatibility
  set(_default OFF)
  if(DEFINED WITH_${NAME} AND NOT WITH_${NAME})
    set(_default ON)
  endif()

  set(_else "(If OFF, the internal version will be used.)")
  option(USE_SYSTEM_${NAME}
    "Use the system-provided copy of ${DESCRIPTION}. ${_else}"
    ${_default})

  # Internally, externals use WITH_<NAME> to decide if an external is enabled,
  # so map the option value to that name.
  if(USE_SYSTEM_${NAME})
    set(WITH_${NAME} OFF CACHE INTERNAL "" FORCE)
  else()
    set(WITH_${NAME} ON CACHE INTERNAL "" FORCE)
  endif()
endfunction()

#------------------------------------------------------------------------------
# Add an option whether or not to build an optional component.
#
# Arguments:
#   <DEFAULT_STATE> - Is the component enabled by default? (`ON` or `OFF`)
#   DEPENDS <expression>
#     A list of expressions which must evaluate to true for the component to
#     be made available. (Otherwise, the component will not be enabled, and the
#     option for the component will be hidden.)
#
# Extra arguments are combined (with a single space) to form the description of
# the option.
#------------------------------------------------------------------------------
function(drake_optional_external NAME DEFAULT_STATE)
  string(REPLACE "\\" "\\\\" _args "${ARGN}")
  cmake_parse_arguments(_opt "" "DEPENDS" "" ${_args})

  foreach(_snippet IN LISTS _opt_UNPARSED_ARGUMENTS)
    set(_description "${_description} ${_snippet}")
  endforeach()
  string(STRIP "${_description}" _description)

  if(DEFINED _opt_DEPENDS)
    cmake_dependent_option(WITH_${NAME}
      ${_description}
      ${DEFAULT_STATE}
      "${_opt_DEPENDS}" OFF)
  else()
    option(WITH_${NAME} ${_description} ${DEFAULT_STATE})
  endif()
endfunction()

###############################################################################
# BEGIN "optional" dependencies

# These are packages that we require and can build, but which we allow the user
# to use their own copy (usually provided by the system) if preferred.

drake_system_dependency(
  EIGEN
  "the Eigen C++ matrix library")

drake_system_dependency(
  GOOGLETEST
  "the Google testing framework")

drake_system_dependency(
  GFLAGS
  "the Google command-line flags processing library")

drake_system_dependency(
  LCM
  "the Lightweight Communications and Marshaling IPC suite")

drake_system_dependency(
  BOT_CORE_LCMTYPES
  "the libbot core LCM types")

# END "optional" dependencies
###############################################################################
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
    "VTK-based visualization tool and robot user interface")

  # Probably not on Windows until lcmgl is split out
  drake_optional_external(LIBBOT ON
    "libbot2 robotics library\;"
    "used for its simple open-gl visualizer + lcmgl for director")

  drake_optional_external(NLOPT ON "Non-linear optimization solver")

  # IPOPT is currently disabled on Mac when MATLAB is enabled due to MATLAB
  # compatibility issues: https://github.com/RobotLocomotion/drake/issues/2578
  drake_optional_external(IPOPT ON
    DEPENDS "NOT APPLE OR NOT MATLAB_EXECUTABLE"
    "Interior Point Optimizer, for solving non-linear optimizations")

  drake_optional_external(OCTOMAP ON
    "3D occupancy mapping library\; provides oct-tree data structures")

  drake_optional_external(SWIG_MATLAB ON
    "A version of SWIG with MATLAB support")
endif()

drake_optional_external(YAML_CPP ON
  "C++ library for reading and writing YAML configuration files")

# END external projects that are ON by default
###############################################################################
# BEGIN external projects that are only needed when MATLAB is in use

# The following projects are default ON when MATLAB is present and enabled.
# Otherwise, they are hidden and default OFF.
drake_optional_external(SPOTLESS ON
  DEPENDS "NOT DISABLE_MATLAB\;MATLAB_EXECUTABLE"
  "polynomial optimization front-end for MATLAB")

# The following projects are default OFF when MATLAB is present and enabled.
# Otherwise, they are hidden and default OFF. Some of them may also be hidden
# on Windows regardless of the status of MATLAB.
drake_optional_external(IRIS OFF
  DEPENDS "NOT DISABLE_MATLAB\;MATLAB_EXECUTABLE\;NOT WIN32\;WITH_MOSEK"
  "fast approximate convex segmentation")

drake_optional_external(SEDUMI OFF
  DEPENDS "NOT DISABLE_MATLAB\;MATLAB_EXECUTABLE\;NOT WIN32"
  "semi-definite programming solver")

drake_optional_external(YALMIP OFF
  DEPENDS "NOT DISABLE_MATLAB\;MATLAB_EXECUTABLE\;NOT WIN32"
  "free optimization front-end for MATLAB")

# END external projects that are only needed when MATLAB is in use
###############################################################################
# BEGIN external projects that are OFF by default

drake_optional_external(SNOPT OFF
  "Sparse Non-linear Optimizer\;"
  "requires access to RobotLocomotion/snopt-pod")

drake_optional_external(SIGNALSCOPE OFF DEPENDS "NOT WIN32\;WITH_DIRECTOR"
  "Live plotting tool for LCM messages")

# Many of these might work on win32 with little or no work... they just haven't
# been tried.
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

  # Almost works on windows;  the update step call to git was complaining about
  # local modifications on drake003
  drake_optional_external(TEXTBOOK OFF
    "The Underactuated Robotics textbook and its examples")

  drake_optional_external(XFOIL OFF
    "use w/ XFOIL to compute aerodynamic coefficients for airfoils")
endif()

# END external projects that are OFF by default
###############################################################################
