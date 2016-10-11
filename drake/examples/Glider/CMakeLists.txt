drake_add_matlab_test(NAME examples/Glider/runDircolPerching OPTIONAL snopt COMMAND runDircolPerching SIZE enormous)
drake_add_matlab_test(NAME examples/Glider/runLQR REQUIRES snopt spotless COMMAND runLQR)
drake_add_matlab_test(NAME examples/Glider/runPSM REQUIRES snopt COMMAND runPSM)
drake_add_matlab_test(NAME examples/Glider/runPassive COMMAND runPassive)

add_subdirectory(test)
