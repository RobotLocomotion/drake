# This is an undocumented internal helper for the FindMatlab
# module ``matlab_add_unit_test`` command.

#=============================================================================
# Copyright 2014-2015 Raffi Enficiaud, Max Planck Society
#
# Distributed under the OSI-approved BSD License (the "License");
# see accompanying file Copyright.txt for details.
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================
# (To distribute this file outside of CMake, substitute the full
#  License text for the above reference.)


# Usage: cmake
#   -Dtest_timeout=180
#   -Doutput_directory=
#   -Dadditional_paths=""
#   -Dno_unittest_framework=""
#   -DMatlab_PROGRAM=matlab_exe_location
#   -DMatlab_ADDITIONNAL_STARTUP_OPTIONS=""
#   -Dtest_name=name_of_the_test
#   -Dcustom_Matlab_test_command=""
#   -Dcmd_to_run_before_test=""
#   -Dunittest_file_to_run
#   -P FindMatlab_TestsRedirect.cmake

# Had to remove -nojvm from here to get this to work.
set(Matlab_UNIT_TESTS_CMD -nosplash -nodesktop -nodisplay ${Matlab_ADDITIONNAL_STARTUP_OPTIONS})
if(WIN32)
  set(Matlab_UNIT_TESTS_CMD ${Matlab_UNIT_TESTS_CMD} -wait)
endif()

if(NOT test_timeout)
  set(test_timeout 180)
endif()

if(NOT cmd_to_run_before_test)
  set(cmd_to_run_before_test)
endif()

get_filename_component(unittest_file_directory   "${unittest_file_to_run}" DIRECTORY)
get_filename_component(unittest_file_to_run_name "${unittest_file_to_run}" NAME_WE)

set(concat_string '${unittest_file_directory}')
message("${additional_paths}")
foreach(s IN LISTS additional_paths)
  if(NOT "${s}" STREQUAL "")
    set(concat_string "${concat_string}, '${s}'")
  endif()
endforeach()

if(CUSTOM_TEST_COMMAND)
  set(unittest_to_run ${CUSTOM_TEST_COMMAND})
else()
  set(unittest_to_run "runtests('${unittest_file_to_run_name}'), exit(max([ans(1,:).Failed]))")
endif()


if(no_unittest_framework)
  set(unittest_to_run "try, ${unittest_file_to_run_name}, catch err, disp('An exception has been thrown during the execution'), disp(err), disp(err.stack), exit(1), end, exit(0)")
endif()

set(Matlab_SCRIPT_TO_RUN
    "addpath(${concat_string}); ${cmd_to_run_before_test}; ${unittest_to_run}")

if(NOT working_directory)
  set(working_directory "${output_directory}")
endif()

string(REPLACE "/" "_" clean_test_name "${test_name}")
set(Matlab_LOG_FILE "${output_directory}/${clean_test_name}.log")

set(devnull)
if(UNIX)
  set(devnull INPUT_FILE /dev/null)
elseif(WIN32)
  set(devnull INPUT_FILE NUL)
endif()

execute_process(
  COMMAND "${Matlab_PROGRAM}" ${Matlab_UNIT_TESTS_CMD} -logfile "${Matlab_LOG_FILE}" -r "${Matlab_SCRIPT_TO_RUN}"
  RESULT_VARIABLE res
  TIMEOUT ${test_timeout}
  OUTPUT_QUIET # we do not want the output twice
  WORKING_DIRECTORY ${working_directory}
  ${devnull}
  )

if(NOT EXISTS ${Matlab_LOG_FILE})
  message( FATAL_ERROR "[MATLAB] ERROR: cannot find the log file ${Matlab_LOG_FILE}")
endif()

# print the output in any case.
file(READ ${Matlab_LOG_FILE} matlab_log_content)
message("Matlab test ${name_of_the_test} output:\n${matlab_log_content}") # if we put FATAL_ERROR here, the file is indented.


if(NOT (res EQUAL 0))
  message( FATAL_ERROR "[MATLAB] TEST FAILED" )
endif()
