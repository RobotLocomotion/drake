include(ExternalProject)

find_package(Git REQUIRED)

if(${CMAKE_GENERATOR} STREQUAL "Unix Makefiles")
  set(MAKE_COMMAND "$(MAKE)") # so we can pass through command line arguments
else()
  find_program(MAKE_COMMAND make)
  if(NOT MAKE_COMMAND)
    message(WARNING "Couldn't find make; non-CMake externals may fail to build")
  endif()
endif()

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

    # Synchronize the submodule url now so parallel updates do not conflict
    # later
    execute_process(
      COMMAND ${GIT_EXECUTABLE} submodule sync -- ${PATH}
      WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
  else()
    set(_sm_UPDATE_COMMAND "")
  endif()

  set(${DOWNLOAD_COMMAND_VAR} ${_sm_DOWNLOAD_COMMAND} PARENT_SCOPE)
  set(${UPDATE_COMMAND_VAR} ${_sm_UPDATE_COMMAND} PARENT_SCOPE)
endfunction()

#------------------------------------------------------------------------------
function(drake_fixup_commands PREFIX)
  foreach(_fc_command ${ARGN})
    if(${PREFIX}_${_fc_command} STREQUAL ":")
      set(${PREFIX}_${_fc_command} ${CMAKE_COMMAND} -E sleep 0 PARENT_SCOPE)
    endif()
  endforeach()
endfunction()

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
function(drake_forceupdate PROJECT)
  # CMake < 3.6 forgets to mark the update step as "ALWAYS" when an explicit
  # UPDATE_COMMAND is used. Add our own "ALWAYS" step to force updates.
  if(AUTO_UPDATE_EXTERNALS AND CMAKE_VERSION VERSION_LESS 3.6)
    ExternalProject_Add_Step(${PROJECT} forceupdate
      DEPENDEES download
      DEPENDERS update
      ALWAYS 1
      )
  endif()
endfunction()

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

  # Capture additional commands for ep_add
  set(_ext_EXTRA_COMMANDS)
  foreach(_command BUILD_COMMAND INSTALL_COMMAND)
    if(DEFINED _ext_${_command})
      list(APPEND _ext_EXTRA_COMMANDS ${_command} "${_ext_${_command}}")
    endif()
  endforeach()

  # Set up the external project build
  ExternalProject_Add(${PROJECT}
    SOURCE_DIR ${_ext_SOURCE_DIR}
    BINARY_DIR ${_ext_BINARY_DIR}
    DOWNLOAD_DIR ${PROJECT_SOURCE_DIR}
    DOWNLOAD_COMMAND ${_ext_DOWNLOAD_COMMAND}
    UPDATE_COMMAND ${_ext_UPDATE_COMMAND}
    ${_ext_EXTRA_COMMANDS}
    INDEPENDENT_STEP_TARGETS update
    BUILD_ALWAYS 1
    DEPENDS ${_ext_deps}
    CMAKE_GENERATOR ${CMAKE_GENERATOR}
    CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
      -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
      -DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}
      -DCMAKE_C_COMPILER:STRING=${CMAKE_C_COMPILER}
      -DCMAKE_C_FLAGS:STRING=${CMAKE_C_FLAGS}
      -DCMAKE_CXX_COMPILER:STRING=${CMAKE_CXX_COMPILER}
      -DCMAKE_CXX_FLAGS:STRING=${CMAKE_CXX_FLAGS}
      -DCMAKE_EXE_LINKER_FLAGS:STRING=${CMAKE_EXE_LINKER_FLAGS}
      -DCMAKE_MODULE_LINKER_FLAGS:STRING=${CMAKE_MODULE_LINKER_FLAGS}
      -DCMAKE_SHARED_LINKER_FLAGS:STRING=${CMAKE_SHARED_LINKER_FLAGS}
      -DCMAKE_STATIC_LINKER_FLAGS:STRING=${CMAKE_STATIC_LINKER_FLAGS}
      ${_ext_VERBOSE}
      ${_ext_CMAKE_ARGS})
endmacro()

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
    if(_ext_BUILD_COMMAND_DIR)
      # FIXME: This is only needed for Director due to the unusual build setup
      #        used by the same. We should fix director to be a 'normal' CMake
      #        project, and remove this option
      set(_ext_BUILD_COMMAND ${CMAKE_COMMAND}
        -E chdir ${_ext_BUILD_COMMAND_DIR}
        ${_ext_BUILD_COMMAND})
    endif()
  endif()

  if(NOT _ext_BUILD_COMMAND STREQUAL "")
    set(_ext_BUILD_ALWAYS 1)
  else()
    set(_ext_BUILD_ALWAYS 0)
  endif()

  ExternalProject_Add(${PROJECT}
    SOURCE_DIR ${_ext_SOURCE_DIR}
    BINARY_DIR ${_ext_BINARY_DIR}
    DOWNLOAD_DIR ${PROJECT_SOURCE_DIR}
    DOWNLOAD_COMMAND "${_ext_DOWNLOAD_COMMAND}"
    UPDATE_COMMAND "${_ext_UPDATE_COMMAND}"
    PATCH_COMMAND "${_ext_PATCH_COMMAND}"
    CONFIGURE_COMMAND "${_ext_CONFIGURE_COMMAND}"
    BUILD_COMMAND "${_ext_BUILD_COMMAND}"
    INSTALL_COMMAND "${_ext_INSTALL_COMMAND}"
    INDEPENDENT_STEP_TARGETS update
    BUILD_ALWAYS ${_ext_BUILD_ALWAYS}
    DEPENDS ${_ext_deps})
endmacro()

#------------------------------------------------------------------------------
function(drake_add_external PROJECT)
  # Parse arguments
  set(_ext_extra_commands
    PATCH_COMMAND
    CONFIGURE_COMMAND
    BUILD_COMMAND
    INSTALL_COMMAND)
  set(_ext_flags LOCAL PUBLIC CMAKE ALWAYS)
  set(_ext_sv_args
    SOURCE_DIR
    BINARY_DIR
    BUILD_COMMAND_DIR # FIXME: fix director then remove this
  )
  set(_ext_mv_args
    CMAKE_ARGS
    REQUIRES
    DEPENDS
    ${_ext_extra_commands}
  )
  cmake_parse_arguments(_ext
    "${_ext_flags}" "${_ext_sv_args}" "${_ext_mv_args}" ${ARGN})
  drake_fixup_commands(_ext ${_ext_extra_commands})

  # Determine if this project is enabled
  string(TOUPPER WITH_${PROJECT} _ext_project_option)
  if(_ext_ALWAYS)
    # Project is always enabled
  elseif(DEFINED ${_ext_project_option})
    if(${_ext_project_option})
      # Project is explicitly enabled
    elseif(WITH_ALL_PUBLIC_EXTERNALS AND DEFINED _ext_PUBLIC)
      # Project is public and all public externals are requested
    elseif(WITH_ALL_SUPPORTED_EXTERNALS)
      # All supported externals are requested
    else()
      # Project is NOT enabled; skip it
      return()
    endif()
  else()
    # Project is not supported on this platform; skip it
    return()
  endif()

  # Check external dependencies of project
  set(_ext_reqs_found TRUE)
  foreach(_ext_req IN LISTS _ext_REQUIRES)
    set(_ext_req_found ${_ext_req}_FOUND)
    string(TOUPPER ${_ext_req_found} _ext_req_found_upper)
    find_package(${_ext_req})
    if(DEFINED ${_ext_req_found} AND ${_ext_req_found})
      # Found requisite; same-case FOUND variable
    elseif(DEFINED ${_ext_req_found_upper} AND ${_ext_req_found_upper})
      # Found requisite; uppercase FOUND variable
    else()
      # Requisite not found
      message(SEND_ERROR "Could not find ${_ext_req}, required for ${PROJECT}")
      set(_ext_reqs_found FALSE)
    endif()
  endforeach()
  if(NOT _ext_reqs_found)
    # One or more required packages was not found
    return()
  endif()

  # Set source directory for external project
  if(NOT DEFINED _ext_SOURCE_DIR)
    set(_ext_SOURCE_DIR ${PROJECT_SOURCE_DIR}/externals/${PROJECT})
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
  if(NOT _ext_LOCAL)
    # Compute the path to the submodule for this external
    file(RELATIVE_PATH _ext_GIT_SUBMODULE_PATH
      ${PROJECT_SOURCE_DIR} ${_ext_SOURCE_DIR})

    # Set up submodule and commands for synchronizing submodule
    drake_add_submodule(${_ext_GIT_SUBMODULE_PATH}
      _ext_DOWNLOAD_COMMAND _ext_UPDATE_COMMAND)
  else()
    # Local "externals" have no download or update step
    set(_ext_DOWNLOAD_COMMAND "")
    set(_ext_UPDATE_COMMAND "")
  endif()

  # Add the external project
  set(_ext_VERBOSE)
  if(_ext_CMAKE)
    drake_add_cmake_external(${PROJECT})
  else()
    drake_add_foreign_external(${PROJECT})
  endif()

  if(NOT _ext_LOCAL)
    # Set up build step to ensure project is updated before build
    drake_forceupdate(${PROJECT})

    # Add external to download dependencies
    add_dependencies(download-all ${PROJECT}-update)
  endif()

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

# TODO: add a custom target for release_filelist

# List of all enabled external projects
set(EXTERNAL_PROJECTS)
