if(lcm_FOUND)
  drake_add_cc_test(pick_and_place_state_machine_test)
  target_link_libraries(pick_and_place_state_machine_test
    drakeLCMTypes
    pickAndPlaceCommon)

  drake_add_cc_test(world_state_test)
  target_link_libraries(world_state_test
    drakeLCMTypes
    pickAndPlaceCommon)
endif()
