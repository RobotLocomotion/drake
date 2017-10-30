# -*- mode: cmake -*-
# vi: set ft=cmake :

drake_matlab_add_unit_test(NAME matlab:call_matlab_client_test
  COMMAND "call_matlab_client('${CMAKE_CURRENT_SOURCE_DIR}/call_matlab_test.rpc')"
)
set_tests_properties(matlab:call_matlab_client_test PROPERTIES REQUIRED_FILES
  "${PROJECT_BINARY_DIR}/call_matlab_client.${Matlab_MEX_EXTENSION}"
)

drake_matlab_add_unit_test(NAME matlab:mathematical_program_test
  COMMAND mathematical_program_test
)
