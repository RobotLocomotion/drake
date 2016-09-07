# drake_add_matlab_test(NAME examples/Valkyrie/runValkyrieBalancing REQUIRES gurobi COMMAND runValkyrieBalancing)  # FIXME: see #2839

if (NOT ${CMAKE_BUILD_TYPE} STREQUAL "Debug" AND UNIX AND NOT APPLE) # FIXME: see #3147.
  drake_add_matlab_test(NAME examples/Valkyrie/runValkyrieBalancingPerturb REQUIRES gurobi lcm libbot COMMAND runValkyrieBalancingPerturb)
endif()

# Deactivated due to memory flakiness in instantaneousQP; see #2165, #2376.
# drake_add_matlab_test(NAME examples/Valkyrie/runValkyrieWalking REQUIRES gurobi lcm libbot COMMAND runValkyrieWalking)

drake_add_matlab_test(NAME examples/Valkyrie/runValkyrieVisualize OPTIONAL bullet gurobi snopt COMMAND runValkyrieVisualize)

add_subdirectory(test)
