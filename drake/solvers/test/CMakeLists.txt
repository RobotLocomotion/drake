if(GTEST_FOUND)
  add_executable(testOptimizationProblem testOptimizationProblem.cpp)
  target_link_libraries(testOptimizationProblem drakeOptimization
    ${GTEST_BOTH_LIBRARIES})
  add_test(NAME testOptimizationProblem COMMAND testOptimizationProblem)

  add_executable(testMobyLCP testMobyLCP.cpp)
  target_link_libraries(testMobyLCP drakeOptimization ${GTEST_BOTH_LIBRARIES})
  add_test(NAME testMobyLCP COMMAND testMobyLCP)

  add_executable(system_identification_test system_identification_test.cpp)
  target_link_libraries(system_identification_test
    drakeOptimization drakePolynomial ${GTEST_BOTH_LIBRARIES})
  add_test(NAME system_identification_test COMMAND system_identification_test)
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
