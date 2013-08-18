
cmake_minimum_required(VERSION 2.8.7 FATAL_ERROR)

set(CTEST_PROJECT_NAME Drake)

if(NOT "${dashboard_model}" MATCHES "^(Nightly|Experimental|Continuous)$")
  message(FATAL_ERROR "dashboard_model must be Nightly, Experimental, or Continuous")
endif()

set(dashboard_source_name "drake-${dashboard_svn_branch}-${dashboard_model}")
set(dashboard_binary_name "build-${dashboard_svn_branch}-${dashboard_model}")
set(CTEST_SOURCE_DIRECTORY ${CTEST_DASHBOARD_ROOT}/${dashboard_source_name})
set(CTEST_BINARY_DIRECTORY ${CTEST_DASHBOARD_ROOT}/${dashboard_binary_name})
set(CTEST_BUILD_COMMAND "bash ${CTEST_SCRIPT_DIRECTORY}/drake_build.sh")
set(CTEST_UPDATE_COMMAND svn)
set(CTEST_TEST_TIMEOUT 1800)
set(CTEST_USE_LAUNCHERS 0)

if(NOT EXISTS "${CTEST_SOURCE_DIRECTORY}")
  # if you set CTEST_CHECKOUT_COMMAND, then ctest can checkout the source
  # from svn automatically, but this often fails do to required server
  # authentication.  So, we won't support automatic checkout.
  #set(CTEST_CHECKOUT_COMMAND "${CTEST_UPDATE_COMMAND} checkout https://svn.csail.mit.edu/locomotion/pods  ${CTEST_SOURCE_DIRECTORY}")
  message(FATAL_ERROR "\nThe svn source directory does not exist.  Checkout with:\n  svn checkout https://svn.csail.mit.edu/locomotion/pods ${CTEST_SOURCE_DIRECTORY}\n")
endif()

set(DRAKE_BASE $ENV{DRAKE_BASE})
if(NOT DEFINED DRAKE_BASE)
  message(FATAL_ERROR "The DRAKE_BASE environment variable is not defined.")
endif()

if(NOT "${CTEST_SOURCE_DIRECTORY}" STREQUAL ${DRAKE_BASE})
  message(FATAL_ERROR "The DRAKE_BASE environment variable does not match the dashboard source directory: ${CTEST_SOURCE_DIRECTORY}")
endif()


#-----------------------------------------------------------------------------

list(APPEND CTEST_NOTES_FILES
  "${CMAKE_CURRENT_LIST_FILE}"
  )


# Print summary information.
foreach(v
    CTEST_SITE
    CTEST_BUILD_NAME
    CTEST_SOURCE_DIRECTORY
    CTEST_BINARY_DIRECTORY
    CTEST_SCRIPT_DIRECTORY
    )
  set(vars "${vars}  ${v}=[${${v}}]\n")
endforeach(v)
message("Dashboard script configuration:\n${vars}\n")

# Avoid non-ascii characters in tool output.
set(ENV{LC_ALL} C)


# Remove all non-svn files in the source directory
set(empty_build_dirs 1)
if(empty_build_dirs)
  execute_process(COMMAND bash "${CTEST_SCRIPT_DIRECTORY}/clean_build_dirs.sh" ${CTEST_SOURCE_DIRECTORY} RESULT_VARIABLE res)
  if(NOT res EQUAL 0)
    message(FATAL_ERROR "Failed to execute clean_build_dirs.sh")
  endif()
endif()

# Remove the ctest build directory
file(REMOVE_RECURSE "${CTEST_BINARY_DIRECTORY}")
file(MAKE_DIRECTORY "${CTEST_BINARY_DIRECTORY}")
file(COPY ${CTEST_SCRIPT_DIRECTORY}/CTestConfig.cmake DESTINATION ${CTEST_BINARY_DIRECTORY})


set(dashboard_continuous 0)
if("${dashboard_model}" STREQUAL "Continuous")
  set(dashboard_continuous 1)
  set(dashboard_continuous_hours 12)
  set(dashboard_continuous_clean_build 1)
  math(EXPR dashboard_continuous_seconds "${dashboard_continuous_hours} * 60 * 60")
endif()

while(TRUE)

  set(dashboard_start_time ${CTEST_ELAPSED_TIME})

  ctest_start(${dashboard_model})

  ctest_update(RETURN_VALUE count)
  message("Found ${count} changed files")

  if(NOT dashboard_continuous OR dashboard_continuous_clean_build OR count GREATER 0)

    # pod builds don't have a separate configure step.
    #ctest_configure()

    ctest_read_custom_files(${CTEST_SCRIPT_DIRECTORY})

    ctest_build()


    if (NOT dashboard_skip_testing)
      # remove old test output file before starting new testing
      file(REMOVE ${CTEST_SOURCE_DIRECTORY}/software/drake/cdash.xml)

      ctest_test(BUILD ${CTEST_SOURCE_DIRECTORY}/software/drake/pod-build)

      # copy xml from drake test results
      file (READ "${CTEST_BINARY_DIRECTORY}/Testing/TAG" tag_file)
      string (REGEX MATCH "[^\n]*" xml_dir ${tag_file})
      set (xml_dir "${CTEST_BINARY_DIRECTORY}/Testing/${xml_dir}")
      message("copying xml data: python ${CTEST_SCRIPT_DIRECTORY}/copy_testing_xml.py ${xml_dir}/Test.xml ${CTEST_SOURCE_DIRECTORY}/software/drake/cdash.xml ${xml_dir}/Test.xml")
      execute_process(COMMAND python ${CTEST_SCRIPT_DIRECTORY}/copy_testing_xml.py ${xml_dir}/Test.xml ${CTEST_SOURCE_DIRECTORY}/software/drake/cdash.xml ${xml_dir}/Test.xml)
    endif()

    if(dashboard_do_coverage)
      ctest_coverage()
    endif()

    if(dashboard_do_memcheck)
      ctest_memcheck()
    endif()

    ctest_submit()

  endif()

  if(dashboard_continuous)
    set(dashboard_continuous_clean_build 0)

    # Delay at least five minutes past the last start time
    ctest_sleep(${dashboard_start_time} 300 ${CTEST_ELAPSED_TIME})
    if(${CTEST_ELAPSED_TIME} GREATER ${dashboard_continuous_seconds})
      return()
    endif()
  else()
    return()
  endif()

endwhile()
