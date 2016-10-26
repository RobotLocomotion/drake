drake_add_matlab_test(NAME systems/test/HybridSystemSampleTimeTest.run COMMAND HybridSystemSampleTimeTest.run)
drake_add_matlab_test(NAME systems/test/LinearGaussianDT.run COMMAND LinearGaussianDT.run)
# drake_add_matlab_test(NAME systems/test/linear_gaussian_dt COMMAND linear_gaussian_dt)  # FIXME: see #366
drake_add_matlab_test(NAME systems/test/memoryTest OPTIONAL bullet COMMAND memoryTest)
# drake_add_matlab_test(NAME systems/test/runPendulum COMMAND runPendulum)  # FIXME: see #310
drake_add_matlab_test(NAME systems/test/testSimInput COMMAND testSimInput)
