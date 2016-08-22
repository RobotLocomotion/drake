add_matlab_test(NAME examples/CompassGait/CompassGaitPlant.run COMMAND CompassGaitPlant.run)
add_matlab_test(NAME examples/CompassGait/runDircolCycle OPTIONAL snopt COMMAND runDircolCycle)

add_subdirectory(test)
