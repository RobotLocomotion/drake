if(lcm_FOUND)
  drake_add_cc_test(robot_state_msg_translator_test)
  target_link_libraries(robot_state_msg_translator_test
      drakeMultibodyParsers
      drakeRobotStateLcmTranslator)
endif()

drake_add_cc_test(trajectory_utils_test)
target_link_libraries(trajectory_utils_test drakeTrajectoryUtils)
