option(TEST_TIMEOUT_MULTIPLIER
  "Positive integer by which to multiply test timeouts" 1)
mark_as_advanced(TEST_TIMEOUT_MULTIPLIER)


#------------------------------------------------------------------------------
# Compute the timeout of a test given its size.
#
# Arguments:
#   <SIZE> - The desired size of the test.
#   <SIZE_OUTVAR> - The actual size of the test.
#   <SIZE_OUTVAR> - The computed timeout of the test.
#------------------------------------------------------------------------------
function(drake_compute_test_timeout SIZE SIZE_OUTVAR TIMEOUT_OUTVAR)
  # http://googletesting.blogspot.com/2010/12/test-sizes.html
  if(SIZE STREQUAL small)
    set(_timeout 60)
  elseif(SIZE STREQUAL medium)
    set(_timeout 300)
  elseif(SIZE STREQUAL large)
    set(_timeout 900)
  elseif(SIZE STREQUAL enormous)
    set(_timeout 2700)
  else()
    set(SIZE small)
    set(_timeout 60)
  endif()

  if(TEST_TIMEOUT_MULTIPLIER GREATER 1)
    math(EXPR _timeout "${_timeout} * ${TEST_TIMEOUT_MULTIPLIER}")
  endif()

  set(${SIZE_OUTVAR} ${SIZE} PARENT_SCOPE)
  set(${TIMEOUT_OUTVAR} ${_timeout} PARENT_SCOPE)
endfunction()


#------------------------------------------------------------------------------
# Add a test to the drake project to be run by ctest.
#
#   drake_add_test(NAME <name> COMMAND <command> [<arg>...]
#                  [CONFIGURATIONS <configuration>...]
#                  [SIZE <small | medium | large | enormous>]
#                  [WORKING_DIRECTORY <directory>])
#
# Arguments:
#   COMMAND
#     Specify the test command-line. If <command> specifies an executable
#     target, it will automatically be replaced by the location of the
#     executable created at build time.
#   CONFIGURATIONS
#     Restrict execution of the test to only the named configurations.
#   SIZE
#     Set the size of the test. Test sizes are primarily defined by the number
#     of seconds to allow for the test execution. If not specified, the size of
#     the test will be set to small. The LABEL test property will be set to the
#     size of the test.
#   WORKING_DIRECTORY
#     Set the WORKING_DIRECTORY test property to specify the working directory
#     in which to execute the test. If not specified, the test will be run with
#     the working directory set to CMAKE_CURRENT_BINARY_DIR.
#------------------------------------------------------------------------------
function(drake_add_test)
  cmake_parse_arguments("" "" "NAME;SIZE" "" ${ARGN})

  if(NOT _SIZE)
    set(_SIZE small)
  endif()

  drake_compute_test_timeout(${_SIZE} _size _timeout)

  if(LONG_RUNNING_TESTS OR _size MATCHES "(small|medium)")
    add_test(NAME ${_NAME} ${_UNPARSED_ARGUMENTS})
    set_tests_properties(${_NAME} PROPERTIES
      LABELS ${_size}
      TIMEOUT ${_timeout})
  else()
    message(STATUS
      "Not running ${_NAME} because ${_size} tests are not enabled")
  endif()
endfunction()


#------------------------------------------------------------------------------
# Add a MATLAB test to the drake project to be run by ctest.
#
#   drake_add_matlab_test(NAME <name> COMMAND <matlab_string_to_evaluate>
#                         [CONFIGURATIONS <configuration>...]
#                         [REQUIRES <external>...]
#                         [OPTIONAL <external>...]
#                         [SIZE <small | medium | large | enormous>]
#                         [WORKING_DIRECTORY <directory>])
#
# Arguments:
#   COMMAND
#     Specify the MATLAB string to evaluate.
#  CONFIGURATIONS
#     Restrict execution of the test to only the named configurations.
#   REQUIRES
#     Declare required external dependencies. Each required dependency must
#     write a file named addpath_<external>.m to CMAKE_INSTALL_PREFIX/matlab or
#     set WITH_<external> or <external>_FOUND to ON. If a required
#     dependency is not available, the test will not be run.
#   OPTIONAL
#     Optional external dependency. Reserved for future use.
#   SIZE
#     Set the size of the test. Test sizes are primarily defined by the number
#     of seconds to allow for the test execution. If not specified, the size of
#     the test will be set to medium. The LABEL test property will be set to
#     the size of the test.
#   WORKING_DIRECTORY
#     Set the WORKING_DIRECTORY test property to specify the working directory
#     in which to execute the test. If not specified, the test will be run with
#     the working directory set to CMAKE_CURRENT_SOURCE_DIR.
#------------------------------------------------------------------------------
function(drake_add_matlab_test)
  if(NOT MATLAB_FOUND)
    return()
  endif()

  cmake_parse_arguments("" "" "COMMAND;NAME;SIZE;WORKING_DIRECTORY" "OPTIONAL;REQUIRES" ${ARGN})

  if(NOT _SIZE)
    set(_SIZE medium)
  endif()

  drake_compute_test_timeout(${_SIZE} _size _timeout)

  if(NOT LONG_RUNNING_TESTS AND NOT _size MATCHES "(small|medium)")
    message(STATUS
      "Not running ${_NAME} because ${_size} tests are not enabled")
    return()
  endif()

  if(_REQUIRES)
    foreach(_require ${_REQUIRES})
      string(TOUPPER ${_require} _require_upper)
      if(NOT WITH_${_require_upper} AND NOT ${_require}_FOUND AND NOT EXISTS "${CMAKE_INSTALL_PREFIX}/matlab/addpath_${_require}.m")
        message(STATUS
          "Not running ${_NAME} because ${_require} was not installed")
        return()
      endif()
    endforeach()
  endif()

  string(REPLACE ' '' _COMMAND ${_COMMAND}) # turn ' into '' so we can eval it in MATLAB
  if(NOT _WORKING_DIRECTORY)
    set(_WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  endif()

  set(_additional_paths
    "${CMAKE_INSTALL_PREFIX}/matlab;${_WORKING_DIRECTORY};")
  set(_test_precommand
    "addpath_drake; global g_disable_visualizers; g_disable_visualizers=true;")

  set(_exit_status
    "~strncmp(ex.identifier,'Drake:MissingDependency',23)")  # FIXME: missing dependency => pass
  set(_test_command
    "try, eval('${_COMMAND}'); catch ex, disp(getReport(ex,'extended')); disp(' '); force_close_system; exit(${_exit_status}); end; force_close_system; exit(0)")

  set(_test_args TEST_ARGS ${_UNPARSED_ARGUMENTS})

  matlab_add_unit_test(
    NAME ${_NAME}
    ADDITIONAL_PATH ${_additional_paths}
    UNITTEST_PRECOMMAND ${_test_precommand}
    CUSTOM_TEST_COMMAND \"${_test_command}\"
    TIMEOUT -1
    WORKING_DIRECTORY ${_WORKING_DIRECTORY}
    ${_test_args})
  set_tests_properties(${_NAME} PROPERTIES
    LABELS ${_size}
    TIMEOUT ${_timeout})
endfunction()


#------------------------------------------------------------------------------
# Deprecated.
#------------------------------------------------------------------------------
function(add_matlab_test)
  message(WARNING
    "The function add_matlab_test is deprecated; use drake_add_matlab_test instead.")
  drake_add_matlab_test(${ARGN})
endfunction()
