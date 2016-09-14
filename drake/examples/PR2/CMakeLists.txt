drake_add_matlab_test(NAME examples/PR2/drawPR2 OPTIONAL bullet COMMAND drawPR2)
# drake_add_matlab_test(NAME examples/PR2/runSaggitalPassive COMMAND runSaggitalPassive)  # FIXME: see #3313
drake_add_matlab_test(NAME examples/PR2/runPassive COMMAND runPassive SIZE large)
