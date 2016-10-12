drake_add_matlab_test(NAME examples/Glider/test/dynamicsGradientsTest COMMAND dynamicsGradientsTest REQUIRES spotless)
drake_add_matlab_test(NAME examples/Glider/test/verifyURDF OPTIONAL bullet COMMAND verifyURDF)
