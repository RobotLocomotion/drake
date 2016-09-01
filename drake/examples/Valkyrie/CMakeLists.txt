if(LONG_RUNNING_TESTS)
  # add_matlab_test(NAME examples/Valkyrie/runValkyrieBalancing REQUIRES gurobi COMMAND runValkyrieBalancing PROPERTIES TIMEOUT 1500)  # FIXME: see #2839

  if (NOT ${CMAKE_BUILD_TYPE} STREQUAL "Debug" AND UNIX AND NOT APPLE) # FIXME: see #3147.
    add_matlab_test(NAME examples/Valkyrie/runValkyrieBalancingPerturb REQUIRES gurobi lcm libbot COMMAND runValkyrieBalancingPerturb PROPERTIES TIMEOUT 1500)
  endif()

  # Deactivated due to memory flakiness in instantaneousQP; see #2165, #2376.
  # add_matlab_test(NAME examples/Valkyrie/runValkyrieWalking REQUIRES gurobi lcm libbot COMMAND runValkyrieWalking PROPERTIES TIMEOUT 1500)
endif()

add_matlab_test(NAME examples/Valkyrie/runValkyrieVisualize OPTIONAL bullet gurobi snopt COMMAND runValkyrieVisualize)

add_subdirectory(test)
