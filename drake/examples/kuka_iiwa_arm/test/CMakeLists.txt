if(lcm_FOUND)
  drake_add_cc_test(iiwa_lcm_test)
  target_link_libraries(iiwa_lcm_test
    drakeKukaIiwaArmCommon
    drakeLCMTypes
    Eigen3::Eigen)
endif()
