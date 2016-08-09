add_matlab_test(NAME examples/Airplane2D/runDircol COMMAND runDircol REQUIRES snopt)
add_matlab_test(NAME examples/Airplane2D/runDircolWithObs COMMAND runDircolWithObs REQUIRES snopt)
add_matlab_test(NAME examples/Airplane2D/runPassive COMMAND runPassive)

add_subdirectory(test)
