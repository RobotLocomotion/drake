drake_add_matlab_test(NAME examples/SimplePulleys/ChineseYoYo.runPassive OPTIONAL bullet snopt COMMAND ChineseYoYo.runPassive)
# drake_add_matlab_test(NAME examples/SimplePulleys/multiple_pulleys COMMAND multiple_pulleys)  # FIXME: see #3313
# drake_add_matlab_test(NAME examples/SimplePulleys/simple_pulley COMMAND simple_pulley)  # FIXME: see #3313
drake_add_matlab_test(NAME examples/SimplePulleys/soft_hand OPTIONAL bullet gurobi snopt COMMAND soft_hand)
drake_add_matlab_test(NAME examples/SimplePulleys/tension OPTIONAL bullet gurobi snopt COMMAND tension)
