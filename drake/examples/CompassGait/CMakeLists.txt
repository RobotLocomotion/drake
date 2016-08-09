add_matlab_test(NAME examples/CompassGait/CompassGaitPlant.run COMMAND CompassGaitPlant.run)
add_matlab_test(NAME examples/CompassGait/runDircolCycle COMMAND runDircolCycle REQUIRES snopt)

add_subdirectory(test)
