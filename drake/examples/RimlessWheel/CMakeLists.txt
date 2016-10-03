drake_add_matlab_test(NAME examples/RimlessWheel/RimlessWheelPlant.run COMMAND RimlessWheelPlant.run)
drake_add_matlab_test(NAME examples/RimlessWheel/runDircol OPTIONAL snopt COMMAND runDircol)
# drake_add_matlab_test(NAME examples/RimlessWheel/runPassiveLCP COMMAND runPassiveLCP)  # FIXME: see #374

add_subdirectory(test)
