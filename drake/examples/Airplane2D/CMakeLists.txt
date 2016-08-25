add_matlab_test(NAME examples/Airplane2D/runDircol OPTIONAL snopt COMMAND runDircol)
add_matlab_test(NAME examples/Airplane2D/runDircolWithObs OPTIONAL snopt COMMAND runDircolWithObs)
add_matlab_test(NAME examples/Airplane2D/runPassive COMMAND runPassive)

add_subdirectory(test)
