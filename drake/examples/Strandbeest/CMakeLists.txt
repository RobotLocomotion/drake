
if (LONG_RUNNING_TESTS)
  add_matlab_test(NAME examples/Strandbeest/runPassiveDownhill COMMAND runPassiveDownhill PROPERTIES TIMEOUT 1500)
  add_matlab_test(NAME examples/Strandbeest/runWithMotor COMMAND runWithMotor PROPERTIES TIMEOUT 1500)
endif()
