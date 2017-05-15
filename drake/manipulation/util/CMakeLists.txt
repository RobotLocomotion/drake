if(lcm_FOUND)
  add_library_with_exports(LIB_NAME drakeRobotStateLcmTranslator SOURCE_FILES
      robot_state_msg_translator.cc)
  target_link_libraries(drakeRobotStateLcmTranslator
      drakeLCMUtil
      drakeGeometryUtil
      drakeRBM)
  drake_install_libraries(drakeRobotStateLcmTranslator)
  drake_install_headers(
      robot_state_msg_translator.h)
  drake_install_pkg_config_file(drake-robot-state-lcm-translator
      TARGET drakeRobotStateLcmTranslator
      LIBS -ldrakeRobotStateLcmTranslator
      REQUIRES drake-geometry-util drake-lcm-util drake-rbm drake-lcmtypes-cpp eigen3)

  add_subdirectory(test)
endif()

add_library_with_exports(LIB_NAME drakeTrajectoryUtils SOURCE_FILES
    trajectory_utils.cc)
target_link_libraries(drakeTrajectoryUtils
    drakeTrajectories)
drake_install_libraries(drakeTrajectoryUtils)
drake_install_headers(
    trajectory_utils.h)
drake_install_pkg_config_file(drake-trajectory-utils
    TARGET drakeTrajectoryUtils
    LIBS -ldrakeTrajectoryUtils
    REQUIRES drake-trajectories)
