if(lcm_FOUND)
  add_library_with_exports(LIB_NAME pickAndPlaceCommon SOURCE_FILES
      action.cc
      pick_and_place_common.cc
      world_state.cc)
  target_link_libraries(pickAndPlaceCommon
      drakeCommon
      drakeIK
      drakeKukaIiwaArmCommon
      drakeKukaIiwaArmIK
      drakeLcmSystem
      drakeLCMTypes
      drakeLCMUtil
      drakeRBM
      drakeMultibodyParsers
      drakeTrajectories
      robotlocomotion-lcmtypes-cpp)
  drake_install_libraries(pickAndPlaceCommon)
  drake_install_headers(
      action.h
      pick_and_place_common.h
      world_state.h)
  drake_install_pkg_config_file(pick-and-place-common
      TARGET pickAndPlaceCommon
      LIBS -lpickAndPlaceCommon
      REQUIRES drake-common drake-ik drake-kuka-iiwa-arm-common
      drake-lcm-system drake-lcmtypes-cpp drake-lcm-utils drake-rbm 
      robotlocomotion-lcmtypes-cpp)


  add_executable(pick_and_place_demo
    pick_and_place_state_machine.cc)
  target_link_libraries(pick_and_place_demo
    drakeKukaIiwaArmCommon
    drakeKukaIiwaArmIK
    drakeLCMTypes
    drakeLCMUtil
    drakeRigidBodyPlant
    pickAndPlaceCommon
    robotlocomotion-lcmtypes-cpp)
endif()

