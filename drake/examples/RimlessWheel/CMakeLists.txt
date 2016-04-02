
add_matlab_test(NAME examples/RimlessWheel/RimlessWheelPlant.run COMMAND RimlessWheelPlant.run)
add_matlab_test(NAME examples/RimlessWheel/runDircol COMMAND runDircol)
add_matlab_test(NAME examples/RimlessWheel/runPassiveLCP COMMAND runPassiveLCP)

add_subdirectory(test)
