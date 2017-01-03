
if (mosek_FOUND AND lcm_FOUND)
  drake_add_cc_test(pose_estimation_test pose_estimation_test.cc)
  target_link_libraries(pose_estimation_test
    drakeRBM
    drakeMultibodyParsers
    drakeOptimization
    drakeLCMTypes
    drakeLcm)
endif(mosek_FOUND and lcm_FOUND)
