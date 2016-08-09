add_matlab_test(NAME examples/Glider/runDircolPerching COMMAND runDircolPerching REQUIRES snopt)
add_matlab_test(NAME examples/Glider/runLQR COMMAND runLQR REQUIRES snopt spotless)
add_matlab_test(NAME examples/Glider/runPSM COMMAND runPSM REQUIRES snopt)
add_matlab_test(NAME examples/Glider/runPassive COMMAND runPassive)

add_subdirectory(test)
