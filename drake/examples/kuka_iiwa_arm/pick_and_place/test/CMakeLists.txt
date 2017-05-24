if(lcm_FOUND)
  drake_add_cc_test(world_state_test)
  target_link_libraries(world_state_test
    pickAndPlaceCommon
    drakeLCMTypes
    Eigen3::Eigen)
endif()
