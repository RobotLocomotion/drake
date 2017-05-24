if(lcm_FOUND)
  add_library_with_exports(LIB_NAME pickAndPlaceTestCommon SOURCE_FILES
      world_state.cc)
  target_link_libraries(pickAndPlaceTestCommon
      drakeCommon
      drakeKukaIiwaArmCommon
      drakeLcmSystem
      drakeLCMTypes
      drakeLCMUtil
      drakeRBM
      drakeMultibodyParsers
      drakeTrajectories
      robotlocomotion-lcmtypes-cpp)
  drake_install_libraries(pickAndPlaceTestCommon)
  drake_install_headers(
      world_state.h)
  drake_install_pkg_config_file(pick-and-place-test-common
      TARGET pickAndPlaceTestCommon
      LIBS -lpickAndPlaceTestCommon
      REQUIRES drake-common
      drake-lcm-system drake-lcmtypes-cpp drake-lcm-utils drake-rbm
      robotlocomotion-lcmtypes-cpp)
endif()
