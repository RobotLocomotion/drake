if(ipopt_FOUND OR NLopt_FOUND OR snopt_FOUND)
  drake_add_cc_test(trajectory_optimization_test)
  target_link_libraries(trajectory_optimization_test drakeTrajectoryOptimization)
endif()
