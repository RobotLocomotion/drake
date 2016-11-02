option(CHECK_DEPENDENCY_STRICT
  "Fail the test if a MATLAB dependency check fails" OFF)
mark_as_advanced(CHECK_DEPENDENCY_STRICT)

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
#   NAME
#     Set the name of the test. The test name may not contain spaces or quotes.
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

  if(NOT _NAME)
    message(FATAL_ERROR "The NAME argument to drake_add_test is required")
  endif()

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
# Add an executable linked to the gtest and gtest-main libraries and add a test
# using the executable to the drake project to be run by ctest.
#
#   drake_add_cc_test(<name>)
#
#   drake_add_cc_test(NAME <name> [EXTENSION <extension>]
#                    [CONFIGURATIONS <configuration>...]
#                    [SIZE <small | medium | large | enormous>]
#                    [WORKING_DIRECTORY <directory>] [EXCLUDE_FROM_ALL])
#
# Arguments:
#   NAME
#     Set the name of the executable and the test. The name may not contain
#     spaces or quotes. The C++ source code for the executable must be
#     completely contained in a single file named <name>.<extension>.
#   EXTENSION
#     Set the source code extension; if not given, use "cc" by default.
#   CONFIGURATIONS
#     Restrict execution of the test to only the named configurations.
#   SIZE
#     Set the size of the test. Test sizes are primarily defined by the number
#     of seconds to allow for the test execution. If not specified, the size of
#     the test will be set to small. The LABEL test property will be set to the
#     size of the test.
#   WORKING_DIRECTORY
#     Set the WORKING_DIRECTORY test property on the test to specify the
#     working directory in which to execute the test. If not specified, the
#     test will be run with the working directory set to
#     CMAKE_CURRENT_BINARY_DIR.
#   EXCLUDE_FROM_ALL
#     Set the EXCLUDE_FROM_ALL target property on the executable to exclude the
#     executable from the default build target.
#------------------------------------------------------------------------------
function(drake_add_cc_test)
  # Parse optional keyword arguments.
  cmake_parse_arguments("" EXCLUDE_FROM_ALL "NAME;EXTENSION;SIZE;WORKING_DIRECTORY" CONFIGURATIONS ${ARGN})

  # Set name from first and only (non-keyword) argument in short signature.
  if(NOT _EXCLUDE_FROM_ALL AND NOT _NAME AND NOT _SIZE AND NOT _WORKING_DIRECTORY AND NOT _CONFIGURATIONS)
    set(_NAME ${ARGV0})
  endif()

  if(NOT _EXTENSION)
    set(_EXTENSION "cc")
  endif()

  # Add the executable and link with gtest and gtest-main.
  if(_EXCLUDE_FROM_ALL)
    set(_exclude_from_all EXCLUDE_FROM_ALL)
  endif()
  add_executable(${_NAME} ${_NAME}.${_EXTENSION} ${_exclude_from_all})
  target_link_libraries(${_NAME} GTest::GTest GTest::Main)

  # Add the test to the project.
  drake_add_test(
    NAME ${_NAME}
    COMMAND ${_NAME}
    CONFIGURATIONS ${_CONFIGURATIONS}
    SIZE ${_SIZE}
    WORKING_DIRECTORY ${_WORKING_DIRECTORY})
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
#   NAME
#     Set the name of the test. The test name may not contain spaces or quotes.
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
  if(NOT Matlab_FOUND)
    return()
  endif()

  cmake_parse_arguments("" "" "COMMAND;NAME;SIZE;WORKING_DIRECTORY" "OPTIONAL;REQUIRES" ${ARGN})

  if(NOT _NAME)
    message(FATAL_ERROR "The NAME argument to drake_add_matlab_test is required")
  endif()

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

  if(CHECK_DEPENDENCY_STRICT)
    set(_exit_status 1)
  else()
    set(_exit_status
      "~strncmp(ex.identifier,'Drake:MissingDependency',23)")  # FIXME: missing dependency => pass
  endif()

  set(_test_command
    "try, eval('${_COMMAND}'); catch ex, disp(getReport(ex,'extended')); disp(' '); force_close_system; exit(${_exit_status}); end; force_close_system; exit(0)")

  set(_test_args TEST_ARGS ${_UNPARSED_ARGUMENTS})

  matlab_add_unit_test(
    NAME ${_NAME}
    ADDITIONAL_PATH "${_additional_paths}"
    UNITTEST_PRECOMMAND "${_test_precommand}"
    CUSTOM_TEST_COMMAND "${_test_command}"
    TIMEOUT -1
    WORKING_DIRECTORY "${_WORKING_DIRECTORY}"
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
