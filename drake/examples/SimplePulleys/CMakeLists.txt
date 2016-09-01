add_matlab_test(NAME examples/SimplePulleys/ChineseYoYo.runPassive OPTIONAL bullet snopt COMMAND ChineseYoYo.runPassive)
# add_matlab_test(NAME examples/SimplePulleys/multiple_pulleys COMMAND multiple_pulleys)  # FIXME
# add_matlab_test(NAME examples/SimplePulleys/simple_pulley COMMAND simple_pulley)  # FIXME
add_matlab_test(NAME examples/SimplePulleys/soft_hand OPTIONAL bullet gurobi snopt COMMAND soft_hand)
add_matlab_test(NAME examples/SimplePulleys/tension OPTIONAL bullet gurobi snopt COMMAND tension)
