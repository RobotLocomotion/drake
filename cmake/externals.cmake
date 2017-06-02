include(ExternalProject)

find_package(Git REQUIRED)

if(CMAKE_GENERATOR STREQUAL "Unix Makefiles")
  set(MAKE_COMMAND "$(MAKE)") # so we can pass through command line arguments
else()
  find_program(MAKE_COMMAND make)
  if(NOT MAKE_COMMAND)
    message(WARNING "Couldn't find make; non-CMake externals may fail to build")
  endif()
endif()

#------------------------------------------------------------------------------
# Initialize a submodule and set the commands to download and update the same.
#------------------------------------------------------------------------------
function(drake_add_submodule PATH DOWNLOAD_COMMAND_VAR UPDATE_COMMAND_VAR)
  # Initialize the submodule configuration now so parallel downloads do not
  # conflict later
  execute_process(
    COMMAND ${GIT_EXECUTABLE} submodule init -- ${PATH}
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})

  # Download externals as Git submodules
  set(_sm_DOWNLOAD_COMMAND
    ${GIT_EXECUTABLE} submodule update --init --recursive -- ${PATH})

  if(AUTO_UPDATE_EXTERNALS)
    set(_sm_UPDATE_COMMAND
      ${CMAKE_COMMAND} -E chdir ${PROJECT_SOURCE_DIR}
      ${_sm_DOWNLOAD_COMMAND})
  else()
    set(_sm_UPDATE_COMMAND "")
  endif()

  set(${DOWNLOAD_COMMAND_VAR} ${_sm_DOWNLOAD_COMMAND} PARENT_SCOPE)
  set(${UPDATE_COMMAND_VAR} ${_sm_UPDATE_COMMAND} PARENT_SCOPE)
endfunction()

#------------------------------------------------------------------------------
# Fixup command arguments given to an external project. This expands delayed
# expansion variables (e.g. `@VAR@) and replaces the no-op token (`:`) with a
# suitable no-op command.
#------------------------------------------------------------------------------
function(drake_fixup_commands PREFIX)
  foreach(_fc_command ${ARGN})
    # Some variables may not be defined unless a particular package or program
    # is found or if a value is specified by the user on the command line. The
    # effect of the variable may be different between being undefined and empty,
    # so do not try to define it.
    if(DEFINED ${PREFIX}_${_fc_command})
      if(${PREFIX}_${_fc_command} STREQUAL ":")
        set(${PREFIX}_${_fc_command} ${CMAKE_COMMAND} -E sleep 0 PARENT_SCOPE)
      else()
        string(CONFIGURE "${${PREFIX}_${_fc_command}}"
               _fc_command_string @ONLY)
        set(${PREFIX}_${_fc_command} "${_fc_command_string}" PARENT_SCOPE)
      endif()
    endif()
  endforeach()
endfunction()

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
# Determine the (enabled) dependencies of an external.
#------------------------------------------------------------------------------
function(drake_compute_dependencies OUTVAR)
  set(_deps)
  foreach(_dep ${ARGN})
    list(FIND EXTERNAL_PROJECTS ${_dep} _index)
    if(NOT _index EQUAL -1)
      list(APPEND _deps ${_dep})
    endif()
  endforeach()
  set(${OUTVAR} ${_deps} PARENT_SCOPE)
endfunction()

#------------------------------------------------------------------------------
# Set up git submodule synchronization for an external project.
#------------------------------------------------------------------------------
function(drake_forceupdate PROJECT)
  if(AUTO_UPDATE_EXTERNALS)
    # CMake < 3.6 forgets to mark the update step as "ALWAYS" when an explicit
    # UPDATE_COMMAND is used. Add our own "ALWAYS" step to force updates.
    if(CMAKE_VERSION VERSION_LESS 3.6)
      ExternalProject_Add_Step(${PROJECT} forceupdate
        DEPENDEES download
        DEPENDERS update
        ALWAYS 1
        )
    endif()
  endif()

  # TODO Once we require CMake 3.7, replace calls to drake_forceupdate with
  # drake_add_submodule_sync_dependency
  drake_add_submodule_sync_dependency(${PROJECT})
endfunction()

#------------------------------------------------------------------------------
# Add a dependency on synchronizing submodules
#------------------------------------------------------------------------------
function(drake_add_submodule_sync_dependency PROJECT)
  if(AUTO_UPDATE_EXTERNALS)
    foreach(_step_target ${PROJECT} ${PROJECT}-update)
      if(TARGET ${_step_target})
        add_dependencies(${_step_target} submodule-sync)
      endif()
    endforeach()
  endif()
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
  if(CMAKE_VERBOSE_MAKEFILE)
    set(_ext_VERBOSE -DCMAKE_VERBOSE_MAKEFILE=ON)
  endif()

  # Capture additional commands for the external project
  set(_ext_EXTRA_COMMANDS)
  foreach(_command BUILD_COMMAND INSTALL_COMMAND)
    if(DEFINED _ext_${_command})
      list(APPEND _ext_EXTRA_COMMANDS ${_command} "${_ext_${_command}}")
    endif()
  endforeach()

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
    Java_JAVA_EXECUTABLE
    Java_JAVAC_EXECUTABLE
    Java_JAVAH_EXECUTABLE
    Java_VERSION_STRING
    CMAKE_JAVA_COMPILE_FLAGS
    CMAKE_MACOSX_RPATH
    CMAKE_INSTALL_RPATH
    CMAKE_INSTALL_RPATH_USE_LINK_PATH
    LIB_SUFFIX)

  if(_ext_FORTRAN)
    list(APPEND _ext_PROPAGATE_CACHE_VARS
      CMAKE_Fortran_COMPILER
      CMAKE_Fortran_FLAGS)

    if(NOT _ext_GENERATOR STREQUAL "Unix Makefiles")
      # Ninja and Xcode may not support Fortran.
      set(_ext_GENERATOR "Unix Makefiles")
    endif()
  endif()

  if(_ext_MATLAB AND Matlab_FOUND)
    list(APPEND _ext_PROPAGATE_CACHE_VARS
      Matlab_ROOT_DIR
      MATLAB_ADDITIONAL_VERSIONS)
  endif()

  if(_ext_PYTHON)
    list(APPEND _ext_PROPAGATE_CACHE_VARS
      PYTHON_EXECUTABLE
      PYTHON_INCLUDE_DIR
      PYTHON_LIBRARY)
  endif()

  if(_ext_QT)
    find_package(Qt 4.8 REQUIRED)
    list(APPEND _ext_PROPAGATE_CACHE_VARS QT_QMAKE_EXECUTABLE)
  endif()

  if(_ext_VTK AND USE_SYSTEM_VTK)
    list(APPEND _ext_PROPAGATE_CACHE_VARS VTK_DIR)
  endif()

  drake_build_cache_args(_ext_PROPAGATE_CACHE ${_ext_LIST_SEPARATOR}
    ${_ext_PROPAGATE_CACHE_VARS})

  # Set up the external project build
  ExternalProject_Add(${PROJECT}
    LIST_SEPARATOR "${_ext_LIST_SEPARATOR}"
    URL ${_ext_URL}
    URL_HASH ${_ext_URL_HASH}
    SOURCE_SUBDIR ${_ext_SOURCE_SUBDIR}
    SOURCE_DIR ${_ext_SOURCE_DIR}
    BINARY_DIR ${_ext_BINARY_DIR}
    DOWNLOAD_DIR ${_ext_DOWNLOAD_DIR}
    DOWNLOAD_COMMAND ${_ext_DOWNLOAD_COMMAND}
    UPDATE_COMMAND ${_ext_UPDATE_COMMAND}
    ${_ext_EXTRA_COMMANDS}
    STEP_TARGETS configure build install
    INDEPENDENT_STEP_TARGETS update
    BUILD_ALWAYS 1
    DEPENDS ${_ext_deps}
    CMAKE_GENERATOR "${_ext_GENERATOR}"
    CMAKE_ARGS
      ${_ext_VERBOSE}
      ${_ext_PROPAGATE_CACHE}
      ${_ext_CMAKE_ARGS})

  if(_ext_TEST)
    file(APPEND ${CMAKE_BINARY_DIR}/CTestExternals.cmake
      "subdirs(\"${_ext_BINARY_DIR}\")\n")
  endif()
endmacro()


#------------------------------------------------------------------------------
# Internal helper to set up an Autotools external project.
#------------------------------------------------------------------------------
macro(drake_add_autotools_external PROJECT)
  if(NOT DEFINED _ext_BINARY_DIR)
    set(_ext_BINARY_DIR ${PROJECT_BINARY_DIR}/externals/${PROJECT})
  endif()

  if(_ext_FORTRAN)
    set(_ext_FORTRAN_ENV
      F77=${CMAKE_Fortran_COMPILER}
      FC=${CMAKE_Fortran_COMPILER}
      FFLAGS=${CMAKE_Fortran_FLAGS})
  else()
    set(_ext_FORTRAN_ENV)
  endif()

  set(_env_command
    ${CMAKE_COMMAND} -E env
    CC=${CMAKE_C_COMPILER}
    CFLAGS=${CMAKE_C_FLAGS}
    CXX=${CMAKE_CXX_COMPILER}
    CXXFLAGS=${CMAKE_CXX_FLAGS}
    LDFLAGS=${CMAKE_SHARED_LINKER_FLAGS}
    ${_ext_FORTRAN_ENV}
    ${_ext_AUTOTOOLS_ENV})

  if(NOT DEFINED _ext_CONFIGURE_COMMAND)
    set(_ext_CONFIGURE_COMMAND
      ${_env_command}
      ${_ext_SOURCE_DIR}/configure
      --prefix=${CMAKE_INSTALL_PREFIX}
      ${_ext_AUTOTOOLS_CONFIGURE_ARGS})
  endif()

  if(CMAKE_VERBOSE_MAKEFILE)
    set(_ext_VERBOSE "V=1")
  else()
    set(_ext_VERBOSE "V=0")
  endif()

  if(NOT DEFINED _ext_BUILD_COMMAND)
    set(_ext_BUILD_COMMAND
      ${_env_command}
      ${MAKE_COMMAND}
      ${_ext_VERBOSE})
  endif()

  if(NOT DEFINED _ext_INSTALL_COMMAND)
    set(_ext_INSTALL_COMMAND
      ${_env_command}
      ${MAKE_COMMAND}
      ${_ext_VERBOSE}
      install)
  endif()

  drake_add_foreign_external(${PROJECT})
endmacro()

#------------------------------------------------------------------------------
# Internal helper to set up a non-CMake external project.
#------------------------------------------------------------------------------
macro(drake_add_foreign_external PROJECT)
  # Set binary directory (default: in-source) for external non-CMake project
  if(NOT _ext_BINARY_DIR)
    set(_ext_BINARY_DIR ${_ext_SOURCE_DIR})
  endif()

  # Set default build command for project
  if(NOT DEFINED _ext_BUILD_COMMAND)
    if(CMAKE_VERBOSE_MAKEFILE)
      set(_ext_VERBOSE "V=1 VERBOSE=1")
    endif()
    set(_ext_BUILD_COMMAND ${MAKE_COMMAND}
      ${_ext_VERBOSE}
      BUILD_PREFIX=${CMAKE_INSTALL_PREFIX}
      BUILD_TYPE=$<CONFIGURATION>)
  endif()

  if(NOT _ext_BUILD_COMMAND STREQUAL "")
    set(_ext_BUILD_ALWAYS 1)
  else()
    set(_ext_BUILD_ALWAYS 0)
  endif()

  ExternalProject_Add(${PROJECT}
    URL ${_ext_URL}
    URL_HASH ${_ext_URL_HASH}
    SOURCE_DIR ${_ext_SOURCE_DIR}
    BINARY_DIR ${_ext_BINARY_DIR}
    DOWNLOAD_DIR ${_ext_DOWNLOAD_DIR}
    DOWNLOAD_COMMAND "${_ext_DOWNLOAD_COMMAND}"
    UPDATE_COMMAND "${_ext_UPDATE_COMMAND}"
    PATCH_COMMAND "${_ext_PATCH_COMMAND}"
    CONFIGURE_COMMAND "${_ext_CONFIGURE_COMMAND}"
    BUILD_COMMAND "${_ext_BUILD_COMMAND}"
    INSTALL_COMMAND "${_ext_INSTALL_COMMAND}"
    STEP_TARGETS configure build install
    INDEPENDENT_STEP_TARGETS update
    BUILD_ALWAYS ${_ext_BUILD_ALWAYS}
    DEPENDS ${_ext_deps})
endmacro()

#------------------------------------------------------------------------------
# Add an external project.
#
# Arguments:
#   LOCAL     - External is local to the source tree (i.e. not a submodule)
#   PUBLIC    - External is public
#   CMAKE     - External uses CMake
#   AUTOTOOLS - External uses Autotools
#   ALWAYS    - External is always built
#   TEST      - External's tests should be included in the superbuild's tests
#   FORTRAN   - External uses Fortran
#   MATLAB    - External uses MATLAB
#   PYTHON    - External uses Python
#   QT        - External uses Qt
#   VTK       - External uses VTK
#
#   REQUIRES <deps...>
#       List of packages (checked via `find_package`) that are required to
#       build the external. Only checked if the external will be built.
#
#   DEPENDS <deps...>
#       List of other external projects on which this external depends. Used to
#       ensure correct build order. (Dependencies which are not enabled will be
#       ignored.)
#
#   SOURCE_SUBDIR <dir> - Specify SOURCE_SUBDIR for external
#   SOURCE_DIR <dir> - Override default SOURCE_DIR for external
#   BINARY_DIR <dir> - Override default BINARY_DIR for external
#   GENERATOR <gen> - Override default CMAKE_GENERATOR for external
#
#   PATCH_COMMAND <args...>
#   CONFIGURE_COMMAND <args...>
#   BUILD_COMMAND <args...>
#   INSTALL_COMMAND <args...>
#       Specify the corresponding command for the external project. Commands
#       with variables like `@VAR@` will expand such variables after the
#       project's dependencies are found (useful if `VAR` is set by finding
#       said dependencies). The special value `:` indicates that the
#       corresponding step should be skipped (the `:` is replaced with a no-op
#       command).
#
#   CMAKE_ARGS <args...>
#       Additional arguments to be passed to the CMake external's CONFIGURE
#       step.
#
#   AUTOTOOLS_ENV <envvars...>
#       Additional environment variables to be passed to the Autotools
#       external's configure command.
#
#   AUTOTOOLS_CONFIGURE_ARGS <args...>
#       Additional arguments to be passed to the Autotools external's configure
#       command.
#
# Arguments with the same name as arguments to `ExternalProject_Add` generally
# have the same function and are passed through.
#------------------------------------------------------------------------------
function(drake_add_external PROJECT)
  # Parse arguments
  set(_ext_extra_commands
    PATCH_COMMAND
    CONFIGURE_COMMAND
    BUILD_COMMAND
    INSTALL_COMMAND)
  set(_ext_flags
    LOCAL
    PUBLIC
    CMAKE
    AUTOTOOLS
    ALWAYS
    TEST
    FORTRAN
    MATLAB
    PYTHON
    QT
    VTK)
  set(_ext_sv_args
    SOURCE_SUBDIR
    SOURCE_DIR
    BINARY_DIR
    GENERATOR
    URL
    URL_HASH
  )
  set(_ext_mv_args
    AUTOTOOLS_CONFIGURE_ARGS
    AUTOTOOLS_ENV
    CMAKE_ARGS
    REQUIRES
    DEPENDS
    ${_ext_extra_commands}
  )
  cmake_parse_arguments(_ext
    "${_ext_flags}" "${_ext_sv_args}" "${_ext_mv_args}" ${ARGN})

  if(NOT _ext_GENERATOR)
    set(_ext_GENERATOR "${CMAKE_GENERATOR}")
  endif()

  string(TOUPPER WITH_${PROJECT} _ext_project_option)
  if(_ext_ALWAYS)
    # Project is "always" enabled, but add a hidden option to turn it off; this
    # is useful for the CI to "build" just google_styleguide in order to run
    # cpplint on the code.
    cmake_dependent_option(
      ${_ext_project_option} "Enable ${PROJECT} (internal)" ON
      "NOT ${_ext_project_option}" ON)
  endif()

  # Determine if this project is enabled
  if(DEFINED ${_ext_project_option})
    if(${_ext_project_option})
      # Project is explicitly enabled
    elseif(WITH_ALL_PUBLIC_EXTERNALS AND DEFINED _ext_PUBLIC)
      # Project is public and all public externals are requested
    elseif(WITH_ALL_SUPPORTED_EXTERNALS)
      # All supported externals are requested
    else()
      # Project is NOT enabled; skip it
      message(STATUS
        "Skipping ${PROJECT} - Please see drake_optional_external (cmake/options.cmake) for "
        "how to enable it.")
      return()
    endif()
  else()
    # Project is not supported on this platform; skip it
    return()
  endif()

  # Check external dependencies of project
  set(_ext_reqs_found TRUE)
  foreach(_ext_req IN LISTS _ext_REQUIRES)
    find_package(${_ext_req})
    if(NOT ${_ext_req}_FOUND)
      message(SEND_ERROR "Could not find ${_ext_req}, required for ${PROJECT}")
      set(_ext_reqs_found FALSE)
    endif()
  endforeach()
  if(NOT _ext_reqs_found)
    # One or more required packages was not found
    return()
  endif()

  # Replace placeholders in commands
  drake_fixup_commands(_ext ${_ext_extra_commands})

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
    set(_ext_DOWNLOAD_DIR ${PROJECT_BINARY_DIR})
  else()
    set(_ext_DOWNLOAD_DIR ${PROJECT_SOURCE_DIR})
  endif()

  # Compute project dependencies
  drake_compute_dependencies(_ext_deps ${_ext_DEPENDS})
  if(NOT DEFINED _ext_deps)
    message(STATUS
      "Preparing to build ${PROJECT}")
  else()
    message(STATUS
      "Preparing to build ${PROJECT} with dependencies ${_ext_deps}")
  endif()

  # Manage updates to the submodule
  if(NOT _ext_LOCAL AND NOT DEFINED _ext_URL)
    # Compute the path to the submodule for this external
    file(RELATIVE_PATH _ext_GIT_SUBMODULE_PATH
      ${PROJECT_SOURCE_DIR} ${_ext_SOURCE_DIR})

    # Set up submodule and commands for synchronizing submodule
    drake_add_submodule(${_ext_GIT_SUBMODULE_PATH}
      _ext_DOWNLOAD_COMMAND _ext_UPDATE_COMMAND)
  elseif(_ext_LOCAL)
    # Local "externals" have no download or update step
    set(_ext_DOWNLOAD_COMMAND "")
    set(_ext_UPDATE_COMMAND "")
  endif()

  # Add the external project
  set(_ext_VERBOSE)
  if(_ext_CMAKE)
    drake_add_cmake_external(${PROJECT})
  elseif(_ext_AUTOTOOLS)
    drake_add_autotools_external(${PROJECT})
  else()
    drake_add_foreign_external(${PROJECT})
  endif()

  if(NOT _ext_LOCAL AND NOT DEFINED _ext_URL)
    # Set up build step to ensure project is updated before build
    drake_forceupdate(${PROJECT})

    # Add external to download dependencies
    add_dependencies(download-all ${PROJECT}-update)
  elseif(CMAKE_GENERATOR STREQUAL "Ninja" AND CMAKE_VERSION VERSION_LESS 3.7)
    # Due to a quirk of how the CMake Ninja generator computes dependencies,
    # all targets that contain a submodule update command, or depend on a
    # target that does, need to depend on the submodule-sync target, or the
    # direct dependency will be lost.
    #
    # TODO remove when we require CMake 3.7
    drake_add_submodule_sync_dependency(${PROJECT})
  endif()

  # Check if project is build-skipped
  string(TOUPPER SKIP_${PROJECT}_BUILD _ext_project_skip_build)
  if(${_ext_project_skip_build})
    # Remove project from ALL, but add configure step to ALL
    set_target_properties(${PROJECT}
      PROPERTIES EXCLUDE_FROM_ALL TRUE)
    set_target_properties(${PROJECT}-configure
      PROPERTIES EXCLUDE_FROM_ALL FALSE)
  endif()

  # Check if project should be skipped due to skipped dependencies
  foreach(_ext_dep ${_ext_deps})
    string(TOUPPER SKIP_${_ext_dep}_BUILD _ext_dep_skip_build)
    if(${_ext_dep_skip_build})
      message(STATUS
        "Excluding ${PROJECT} from ALL because build of dependency "
        "${_ext_dep} is skipped or excluded")

      # Remove project from ALL
      set_target_properties(${PROJECT} ${PROJECT}-configure
        PROPERTIES EXCLUDE_FROM_ALL TRUE)

      # Mark project as skipped for this build only (not cached) so downstream
      # dependencies (if any) will also be skipped
      set(${_ext_project_skip_build} ON PARENT_SCOPE)
      break()
    endif()
  endforeach()

  # Add extra per-project targets
  add_custom_target(status-${PROJECT}
    COMMAND ${GIT_EXECUTABLE} status
    WORKING_DIRECTORY ${_ext_SOURCE_DIR})
  add_dependencies(status status-${PROJECT})

  # Register project in list of external projects
  list(APPEND EXTERNAL_PROJECTS ${PROJECT})
  set(EXTERNAL_PROJECTS ${EXTERNAL_PROJECTS} PARENT_SCOPE)
endfunction()

###############################################################################

# Options controlling external dependencies
option(AUTO_UPDATE_EXTERNALS
  "Update external projects to their tag revision on compile"
  ON)

option(WITH_ALL_PUBLIC_EXTERNALS
  "Enable all externals available to public academic users"
  OFF)

option(WITH_ALL_SUPPORTED_EXTERNALS
  "Enable all externals available for your platform (includes private git repositories)"
  OFF)

# Special targets
add_custom_target(download-all)
add_custom_target(status)

if(AUTO_UPDATE_EXTERNALS)
  # Set up a special target to synchronize all submodules. We do this
  # separately to avoid conflicting updates that could result if we tried to
  # synchronize multiple submodules in parallel.
  add_custom_target(submodule-sync
    COMMAND ${GIT_EXECUTABLE} submodule --quiet sync --
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
endif()

# TODO: add a custom target for release_filelist

# List of all enabled external projects
set(EXTERNAL_PROJECTS)
