if(lcm_FOUND)
  add_library_with_exports(LIB_NAME pickAndPlaceCommon SOURCE_FILES
      action.cc
      pick_and_place_state_machine.cc
      world_state.cc)
  target_link_libraries(pickAndPlaceCommon
      drakeCommon
      drakeKukaIiwaArmCommon
      drakeLCMTypes
      drakeLCMUtil
      drakeManipulationPlanner
      drakeMultibodyParsers
      drakeRBM
      drakeTrajectories
      robotlocomotion-lcmtypes-cpp)
  drake_install_libraries(pickAndPlaceCommon)
  drake_install_headers(
      action.h
      pick_and_place_state_machine.h
      world_state.h)
  drake_install_pkg_config_file(pick-and-place-common
      TARGET pickAndPlaceCommon
      LIBS -lpickAndPlaceCommon
      REQUIRES drake-common
      drake-lcmtypes-cpp drake-lcm-utils drake-rbm
      robotlocomotion-lcmtypes-cpp)
endif()

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
