drake_add_matlab_test(NAME examples/CompassGait/CompassGaitPlant.run COMMAND CompassGaitPlant.run)
drake_add_matlab_test(NAME examples/CompassGait/runDircolCycle OPTIONAL snopt COMMAND runDircolCycle)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
