add_matlab_test(NAME examples/SimplePulleys/ChineseYoYo.runPassive COMMAND ChineseYoYo.runPassive REQUIRES snopt)
# add_matlab_test(NAME examples/SimplePulleys/multiple_pulleys COMMAND multiple_pulleys)  # FIXME
# add_matlab_test(NAME examples/SimplePulleys/simple_pulley COMMAND simple_pulley)  # FIXME
add_matlab_test(NAME examples/SimplePulleys/soft_hand COMMAND soft_hand REQUIRES gurobi snopt)
add_matlab_test(NAME examples/SimplePulleys/tension COMMAND tension REQUIRES gurobi snopt)
