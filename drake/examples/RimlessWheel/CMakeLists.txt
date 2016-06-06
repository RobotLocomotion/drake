add_matlab_test(NAME examples/RimlessWheel/RimlessWheelPlant.run COMMAND RimlessWheelPlant.run)
add_matlab_test(NAME examples/RimlessWheel/runDircol COMMAND runDircol)
# add_matlab_test(NAME examples/RimlessWheel/runPassiveLCP COMMAND runPassiveLCP)  # FIXME: see #374

add_subdirectory(test)
