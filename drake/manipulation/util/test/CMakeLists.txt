if(lcm_FOUND)
  drake_add_cc_test(robot_state_msg_translator_test)
  target_link_libraries(robot_state_msg_translator_test
      drakeMultibodyParsers
      drakeRobotStateLcmTranslator)
endif()
