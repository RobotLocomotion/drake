add_matlab_test(NAME examples/SimpleFourBar/runPassive COMMAND runPassive REQUIRES snopt)
add_matlab_test(NAME examples/SimpleFourBar/runPassiveLCP COMMAND runPassiveLCP REQUIRES gurobi snopt)
