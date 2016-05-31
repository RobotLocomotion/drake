add_matlab_test(NAME examples/PR2/drawPR2 COMMAND drawPR2)
# add_matlab_test(NAME examples/PR2/runSaggitalPassive COMMAND runSaggitalPassive)  # FIXME

if(LONG_RUNNING_TESTS)
  add_matlab_test(NAME examples/PR2/runPassive COMMAND runPassive PROPERTIES TIMEOUT 3000)
endif()
