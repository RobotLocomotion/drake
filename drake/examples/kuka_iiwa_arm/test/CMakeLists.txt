if(lcm_FOUND)
  drake_add_cc_test(NAME iiwa_ik_test SIZE medium)
  target_link_libraries(iiwa_ik_test drakeKukaIiwaArmIK)
endif()
