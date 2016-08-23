add_matlab_test(NAME examples/UnderwaterAcrobot/runLQR COMMAND runLQR)
add_matlab_test(NAME examples/UnderwaterAcrobot/runPassive COMMAND runPassive)
add_matlab_test(NAME examples/UnderwaterAcrobot/runPassiveURDF OPTIONAL bullet COMMAND runPassiveURDF)
add_matlab_test(NAME examples/UnderwaterAcrobot/runSwingUp OPTIONAL snopt COMMAND runSwingUp)
add_matlab_test(NAME examples/UnderwaterAcrobot/runURDFandAnalyticSoln OPTIONAL bullet COMMAND runURDFandAnalyticSoln)

add_subdirectory(test)
