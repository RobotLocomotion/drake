add_matlab_test(NAME matlab/solvers/test/gloptipoly_constrained_test REQUIRES gloptipoly3 sedumi  spotless COMMAND gloptipoly_constrained_test)
add_matlab_test(NAME matlab/solvers/test/rrt_basic COMMAND rrt_basic)
add_matlab_test(NAME matlab/solvers/test/rrt_bugtrap COMMAND rrt_bugtrap)
add_matlab_test(NAME matlab/solvers/test/rrt_piano_mover REQUIRES lcm libbot OPTIONAL bullet COMMAND rrt_piano_mover)
add_matlab_test(NAME matlab/solvers/test/rrt_piano_mover_rotation REQUIRES lcm libbot OPTIONAL bullet COMMAND rrt_piano_mover_rotation)
add_matlab_test(NAME matlab/solvers/test/rrt_piano_mover_translation REQUIRES lcm libbot OPTIONAL bullet COMMAND rrt_piano_mover_translation)
# add_matlab_test(NAME matlab/solvers/test/sixHumpCamel REQUIRES spotless OPTIONAL bertini gloptipoly3 snopt COMMAND sixHumpCamel)  # FIXME
add_matlab_test(NAME matlab/solvers/test/testConstraint COMMAND testConstraint)
add_matlab_test(NAME matlab/solvers/test/testGurobiLCP REQUIRES gurobi COMMAND testGurobiLCP)
add_matlab_test(NAME matlab/solvers/test/testLQRmex OPTIONAL bullet COMMAND testLQRmex)
add_matlab_test(NAME matlab/solvers/test/testMixedIntegerConvexDemo REQUIRES gurobi yalmip COMMAND testMixedIntegerConvexDemo)
# DISABLED due to #3149.
# add_matlab_test(NAME matlab/solvers/test/testMultipleTaskSpaceRRTStar REQUIRES lcm OPTIONAL bullet COMMAND testMultipleTaskSpaceRRTStar PROPERTIES TIMEOUT 750)
add_matlab_test(NAME matlab/solvers/test/testNLP OPTIONAL ipopt snopt COMMAND testNLP)
add_matlab_test(NAME matlab/solvers/test/testQP OPTIONAL gurobi snopt COMMAND testQP)
# DISABLED due to #3149.
# add_matlab_test(NAME matlab/solvers/test/testTaskSpaceRRT REQUIRES lcm snopt OPTIONAL bullet COMMAND testTaskSpaceRRT PROPERTIES TIMEOUT 750)
