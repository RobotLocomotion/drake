# drake_add_matlab_test(NAME examples/Valkyrie/runValkyrieBalancing REQUIRES gurobi COMMAND runValkyrieBalancing)  # FIXME: see #2839

drake_add_matlab_test(NAME examples/Valkyrie/runValkyrieBalancingPerturb REQUIRES gurobi lcm libbot yaml-cpp COMMAND runValkyrieBalancingPerturb)

drake_add_matlab_test(NAME examples/Valkyrie/runValkyrieWalking REQUIRES gurobi lcm libbot yaml-cpp COMMAND runValkyrieWalking)

drake_add_matlab_test(NAME examples/Valkyrie/runValkyrieVisualize OPTIONAL bullet gurobi snopt COMMAND runValkyrieVisualize)

add_subdirectory(test)
