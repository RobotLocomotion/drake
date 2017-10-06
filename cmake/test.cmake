#------------------------------------------------------------------------------
# Add a MATLAB test to the drake project to be run by ctest.
#
#   drake_add_matlab_test(NAME <name> COMMAND <matlab_string_to_evaluate>
#                         [SIZE <small | medium | large | enormous>]
#                         [WORKING_DIRECTORY <directory>])
#
# Arguments:
#   NAME
#     Set the name of the test. The test name may not contain spaces or quotes.
#   COMMAND
#     Specify the MATLAB string to evaluate.
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
  cmake_parse_arguments("" "" "COMMAND;NAME;SIZE;WORKING_DIRECTORY" "" ${ARGN})

  if(NOT _NAME)
    message(FATAL_ERROR "The NAME argument to drake_add_matlab_test is required")
  endif()

  # http://googletesting.blogspot.com/2010/12/test-sizes.html
  if(NOT _SIZE)
    set(_SIZE medium)
  endif()
  if(_SIZE STREQUAL small)
    set(_TIMEOUT 60)
  elseif(_SIZE STREQUAL medium)
    set(_TIMEOUT 300)
  elseif(_SIZE STREQUAL large)
    set(_TIMEOUT 900)
  elseif(_SIZE STREQUAL enormous)
    set(_TIMEOUT 2700)
  else()
    message(FATAL_ERROR "The SIZE argument to drake_add_matlab_test is invalid")
  endif()

  string(REPLACE ' '' _COMMAND ${_COMMAND}) # turn ' into '' so we can eval it in MATLAB
  if(NOT _WORKING_DIRECTORY)
    set(_WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  endif()

  set(_ADDITIONAL_PATH "${CMAKE_SOURCE_DIR}/matlab;${_WORKING_DIRECTORY}")

  set(_CUSTOM_TEST_COMMAND
    "try, addpath_drake; eval('${_COMMAND}'); catch ex, disp(getReport(ex, 'extended')); disp(' '); exit(1); end; exit(0)")

  set(_TEST_ARGS TEST_ARGS ${_UNPARSED_ARGUMENTS})

  matlab_add_unit_test(
    NAME ${_NAME}
    ADDITIONAL_PATH "${_ADDITIONAL_PATH}"
    CUSTOM_TEST_COMMAND "${_CUSTOM_TEST_COMMAND}"
    TIMEOUT -1
    WORKING_DIRECTORY "${_WORKING_DIRECTORY}"
    ${_TEST_ARGS})
  set_tests_properties(${_NAME} PROPERTIES
    LABELS ${_SIZE}
    TIMEOUT ${_TIMEOUT})
endfunction()
