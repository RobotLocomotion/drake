function(drake_matlab_add_mex NAME)
  matlab_add_mex(NAME ${NAME} SHARED SRC ${ARGN})

  set_target_properties(${NAME} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}"
  )

  foreach(_CONFIGURATION_TYPE ${CMAKE_CONFIGURATION_TYPES})
    string(TOUPPER "${_CONFIGURATION_TYPE}" _CONFIGURATION_TYPE_UPPER)
    set_target_properties(${NAME} PROPERTIES
      LIBRARY_OUTPUT_DIRECTORY_${_CONFIGURATION_TYPE_UPPER}
        "${PROJECT_BINARY_DIR}"
    )
  endforeach()
endfunction()

function(drake_matlab_add_unit_test)
  cmake_parse_arguments("" "" "COMMAND;NAME;SIZE" "" ${ARGN})

  if(NOT _NAME)
    message(FATAL_ERROR
      "The NAME argument to drake_matlab_add_unit_test is required"
    )
  endif()

  string(REPLACE "'" "''" _COMMAND ${_COMMAND})

  set(_CUSTOM_TEST_COMMAND
    "try, addpath_drake; eval('${_COMMAND}'); catch ex, disp(getReport(ex, 'extended')); disp(' '); exit(1); end; exit(0)"
  )

  set(_TEST_ARGS TEST_ARGS ${_UNPARSED_ARGUMENTS})

  matlab_add_unit_test(
    NAME ${_NAME}
    ADDITIONAL_PATH "${CMAKE_CURRENT_SOURCE_DIR};${PROJECT_BINARY_DIR}"
    CUSTOM_TEST_COMMAND "${_CUSTOM_TEST_COMMAND}"
    TIMEOUT -1
    WORKING_DIRECTORY "${PROJECT_BINARY_DIR}"
    ${_TEST_ARGS}
  )

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
    message(FATAL_ERROR
      "The SIZE argument to drake_matlab_add_unit_test is invalid"
    )
  endif()

  set(_COST ${_TIMEOUT})
  set(_LABELS ${_SIZE})

  set_tests_properties(${_NAME} PROPERTIES
    COST ${_TIMEOUT}
    LABELS ${_SIZE}
    TIMEOUT ${_TIMEOUT}
  )
endfunction()
