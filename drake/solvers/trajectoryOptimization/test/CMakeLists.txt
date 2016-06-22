if(GTEST_FOUND)
  add_executable(trajectory_optimization_test trajectory_optimization_test.cc)
  target_link_libraries(trajectory_optimization_test drakeTrajectoryOptimization
    drakeOptimization ${GTEST_BOTH_LIBRARIES})
  add_test(NAME trajectory_optimization_test COMMAND trajectory_optimization_test)
endif()

add_matlab_test(NAME solvers/trajectoryOptimization/test/contactImplicitBrick COMMAND contactImplicitBrick PROPERTIES TIMEOUT 750)
