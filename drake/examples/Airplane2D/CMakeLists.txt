drake_add_matlab_test(NAME examples/Airplane2D/runDircol OPTIONAL snopt COMMAND runDircol)
drake_add_matlab_test(NAME examples/Airplane2D/runDircolWithObs REQUIRES snopt COMMAND runDircolWithObs)
drake_add_matlab_test(NAME examples/Airplane2D/runPassive COMMAND runPassive)

add_subdirectory(test)
