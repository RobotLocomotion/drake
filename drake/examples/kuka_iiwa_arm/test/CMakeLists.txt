if(lcm_FOUND)
  drake_add_cc_test(rigid_body_plant_mux_test)
  target_link_libraries(rigid_body_plant_mux_test
    drakeKukaIiwaArmCommon
    drakeSystemPrimitives)
endif()
