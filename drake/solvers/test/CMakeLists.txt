add_executable(optimization_problem_test optimization_problem_test.cc)
target_link_libraries(optimization_problem_test drakeOptimization
  ${GTEST_BOTH_LIBRARIES})
add_test(NAME optimization_problem_test COMMAND optimization_problem_test)

if(mosek_FOUND)
  add_executable(mosek_test mosek_test.cc)
  target_link_libraries(mosek_test drakeOptimization ${GTEST_BOTH_LIBRARIES})
  add_test(NAME mosek_test COMMAND mosek_test)
endif()

add_executable(moby_lcp_solver_test moby_lcp_solver_test.cc)
target_link_libraries(moby_lcp_solver_test drakeOptimization ${GTEST_BOTH_LIBRARIES})
add_test(NAME moby_lcp_solver_test COMMAND moby_lcp_solver_test)

add_executable(system_identification_test system_identification_test.cc)
target_link_libraries(system_identification_test
  drakeOptimization drakeCommon ${GTEST_BOTH_LIBRARIES})
add_test(NAME system_identification_test COMMAND system_identification_test)
add_executable(gurobi_solver_test gurobi_solver_test.cc)
target_link_libraries(gurobi_solver_test drakeOptimization
  ${GTEST_BOTH_LIBRARIES})

if(gurobi_FOUND)
  add_test(NAME gurobi_solver_test COMMAND gurobi_solver_test)
endif()

add_matlab_test(NAME solvers/test/gloptipoly_constrained_test COMMAND gloptipoly_constrained_test)
add_matlab_test(NAME solvers/test/rrt_basic COMMAND rrt_basic)
add_matlab_test(NAME solvers/test/rrt_bugtrap COMMAND rrt_bugtrap)
add_matlab_test(NAME solvers/test/rrt_piano_mover COMMAND rrt_piano_mover)
add_matlab_test(NAME solvers/test/rrt_piano_mover_rotation COMMAND rrt_piano_mover_rotation)
add_matlab_test(NAME solvers/test/rrt_piano_mover_translation COMMAND rrt_piano_mover_translation)
# add_matlab_test(NAME solvers/test/sixHumpCamel COMMAND sixHumpCamel)  # FIXME
add_matlab_test(NAME solvers/test/testConstraint COMMAND testConstraint)
add_matlab_test(NAME solvers/test/testGurobiLCP COMMAND testGurobiLCP)
add_matlab_test(NAME solvers/test/testLQRmex COMMAND testLQRmex)
add_matlab_test(NAME solvers/test/testMixedIntegerConvexDemo COMMAND testMixedIntegerConvexDemo)
add_matlab_test(NAME solvers/test/testMultipleTaskSpaceRRTStar COMMAND testMultipleTaskSpaceRRTStar PROPERTIES TIMEOUT 750)
add_matlab_test(NAME solvers/test/testNLP COMMAND testNLP)
add_matlab_test(NAME solvers/test/testQP COMMAND testQP)
add_matlab_test(NAME solvers/test/testTaskSpaceRRT COMMAND testTaskSpaceRRT PROPERTIES TIMEOUT 750)
