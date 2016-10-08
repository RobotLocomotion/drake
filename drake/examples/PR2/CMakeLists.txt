drake_add_matlab_test(NAME examples/PR2/drawPR2 OPTIONAL bullet COMMAND drawPR2)
# drake_add_matlab_test(NAME examples/PR2/runSaggitalPassive COMMAND runSaggitalPassive)  # FIXME: see #3313
# Attempt to only add the test when not in the OPEN_SOURCE CI configuration
# FIXME: see #3444
if(WITH_IRIS OR WITH_SEDUMI)
  drake_add_matlab_test(NAME examples/PR2/runPassive COMMAND runPassive SIZE large)
endif()
