
if (LONG_RUNNING_TESTS)
  add_matlab_test(NAME examples/Valkyrie/runValkyrieBalancing COMMAND runValkyrieBalancing)
  add_matlab_test(NAME examples/Valkyrie/runValkyrieBalancingPerturb COMMAND runValkyrieBalancingPerturb)
  add_matlab_test(NAME examples/Valkyrie/runValkyrieWalking COMMAND runValkyrieWalking)
endif()

add_matlab_test(NAME examples/Valkyrie/runValkyrieVisualize COMMAND runValkyrieVisualize)

add_subdirectory(test)
