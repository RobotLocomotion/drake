if(lcm_FOUND)
  add_library_with_exports(LIB_NAME pickAndPlaceDevCommon SOURCE_FILES
      pick_and_place_common.cc)
  target_link_libraries(pickAndPlaceDevCommon
      drakeCommon
      drakeIK
      drakeKukaIiwaArmCommon
      drakeManipulationPlanner
      drakeLCMTypes
      drakeLCMUtil
      drakeRBM
      drakeMultibodyParsers
      drakeTrajectories
      robotlocomotion-lcmtypes-cpp)
  drake_install_libraries(pickAndPlaceDevCommon)
  drake_install_headers(
      pick_and_place_common.h)
  drake_install_pkg_config_file(pick-and-place-dev-common
      TARGET pickAndPlaceDevCommon
      LIBS -lpickAndPlaceDevCommon
      REQUIRES drake-common drake-ik drake-kuka-iiwa-arm-common
      drake-lcm-system drake-lcmtypes-cpp drake-lcm-utils drake-rbm
      robotlocomotion-lcmtypes-cpp)


  add_executable(pick_and_place_demo
    pick_and_place_demo.cc)
  target_link_libraries(pick_and_place_demo
    drakeKukaIiwaArmCommon
    drakeManipulationPlanner
    drakeLCMTypes
    drakeLCMUtil
    drakeRigidBodyPlant
    pickAndPlaceDevCommon
    pickAndPlaceCommon
    robotlocomotion-lcmtypes-cpp)
endif()
