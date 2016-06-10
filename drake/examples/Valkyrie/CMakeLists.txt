if(LONG_RUNNING_TESTS)
  add_matlab_test(NAME examples/Valkyrie/runValkyrieBalancing COMMAND runValkyrieBalancing PROPERTIES TIMEOUT 1500)
  add_matlab_test(NAME examples/Valkyrie/runValkyrieBalancingPerturb COMMAND runValkyrieBalancingPerturb PROPERTIES TIMEOUT 1500)
  # Deactivated due to memory flakiness in instantaneousQP; see #2165, #2376.
  # add_matlab_test(NAME examples/Valkyrie/runValkyrieWalking COMMAND runValkyrieWalking PROPERTIES TIMEOUT 1500)
endif()

add_matlab_test(NAME examples/Valkyrie/runValkyrieVisualize COMMAND runValkyrieVisualize)

add_subdirectory(test)
