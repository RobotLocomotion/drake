drake_add_matlab_test(NAME examples/Glider/runDircolPerching OPTIONAL snopt COMMAND runDircolPerching SIZE enormous)
drake_add_matlab_test(NAME examples/Glider/runLQR REQUIRES spotless OPTIONAL snopt COMMAND runLQR)
drake_add_matlab_test(NAME examples/Glider/runPSM OPTIONAL snopt COMMAND runPSM)
drake_add_matlab_test(NAME examples/Glider/runPassive COMMAND runPassive)

add_subdirectory(test)
