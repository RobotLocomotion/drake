drake_add_matlab_test(NAME examples/SimpleFourBar/runPassive OPTIONAL bullet snopt COMMAND runPassive)
drake_add_matlab_test(NAME examples/SimpleFourBar/runPassiveLCP OPTIONAL bullet gurobi snopt COMMAND runPassiveLCP)
