drake_add_matlab_test(NAME examples/TwoWheeledInvertedPendulum/runObserverControl COMMAND runObserverControl)
drake_add_matlab_test(NAME examples/TwoWheeledInvertedPendulum/runSwingUp OPTIONAL snopt COMMAND runSwingUp)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
