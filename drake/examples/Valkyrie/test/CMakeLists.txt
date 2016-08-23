if(LONG_RUNNING_TESTS)
  # Deactivated due to memory flakiness in instantaneousQP; see #2165, #2376.
  # add_matlab_test(NAME examples/Valkyrie/test/runValkyrieFastWalking REQUIRES gurobi lcm COMMAND runValkyrieFastWalking PROPERTIES TIMEOUT 1500)
  # add_matlab_test(NAME examples/Valkyrie/test/runValkyrieTurning REQUIRES gurobi lcm COMMAND runValkyrieTurning PROPERTIES TIMEOUT 1500)
  # add_matlab_test(NAME examples/Valkyrie/test/runValkyrieTurning270 REQUIRES gurobi lcm COMMAND runValkyrieTurning270 PROPERTIES TIMEOUT 1500)
endif()
