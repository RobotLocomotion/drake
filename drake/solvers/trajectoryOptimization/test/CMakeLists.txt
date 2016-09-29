add_executable(trajectory_optimization_test trajectory_optimization_test.cc)
target_link_libraries(trajectory_optimization_test drakeTrajectoryOptimization
  ${GTEST_BOTH_LIBRARIES})
# TODO(Lucy-tri) There is currently no non-linear solver on Windows: see
# issues #2352, #2569.
find_package(NLopt)
if(ipopt_FOUND OR NLopt_FOUND OR snopt_c_FOUND)
  drake_add_test(NAME trajectory_optimization_test COMMAND
    trajectory_optimization_test)
endif()
