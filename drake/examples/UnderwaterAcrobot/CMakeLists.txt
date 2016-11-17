drake_add_matlab_test(NAME examples/UnderwaterAcrobot/runLQR COMMAND runLQR)
drake_add_matlab_test(NAME examples/UnderwaterAcrobot/runPassive COMMAND runPassive)
drake_add_matlab_test(NAME examples/UnderwaterAcrobot/runPassiveURDF OPTIONAL bullet COMMAND runPassiveURDF)
drake_add_matlab_test(NAME examples/UnderwaterAcrobot/runSwingUp OPTIONAL snopt COMMAND runSwingUp)
drake_add_matlab_test(NAME examples/UnderwaterAcrobot/runURDFandAnalyticSoln OPTIONAL bullet COMMAND runURDFandAnalyticSoln)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
