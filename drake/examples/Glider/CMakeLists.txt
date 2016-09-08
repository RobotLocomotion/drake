add_matlab_test(NAME examples/Glider/runDircolPerching OPTIONAL snopt COMMAND runDircolPerching SIZE enormous)
add_matlab_test(NAME examples/Glider/runLQR REQUIRES spotless OPTIONAL snopt COMMAND runLQR)
add_matlab_test(NAME examples/Glider/runPSM OPTIONAL snopt COMMAND runPSM)
add_matlab_test(NAME examples/Glider/runPassive COMMAND runPassive)

add_subdirectory(test)
