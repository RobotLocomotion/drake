if(lcm_FOUND AND mosek_FOUND)
  drake_add_cc_test(pose_estimation_test)
  target_link_libraries(pose_estimation_test drakeRBM drakeMultibodyParsers drakeOptimization lcm)
endif()
