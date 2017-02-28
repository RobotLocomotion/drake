if(lcm_FOUND)
  add_executable(pick_and_place_demo
    action.cc
    pick_and_place_state_machine.cc
    world_state.cc)
  target_link_libraries(pick_and_place_demo
    drakeKukaIiwaArmCommon
    drakeKukaIiwaArmIK
    drakeLCMUtil
    drakeRigidBodyPlant
    drakeSchunkWsg
    robotlocomotion-lcmtypes-cpp)
endif()

