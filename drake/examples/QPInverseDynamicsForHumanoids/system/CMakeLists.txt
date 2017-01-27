if(gurobi_FOUND AND lcm_FOUND AND yaml-cpp_FOUND)
  add_executable(valkyrie_balancing_controller_system
    joint_level_controller_system.cc
    plan_eval_system.cc
    qp_controller_system.cc
    robot_state_decoder_system.cc
    valkyrie_balancing_controller_system.cc)
  target_link_libraries(valkyrie_balancing_controller_system
    drakeLCMSystem2
    drakeMultibodyParsers
    drakeQPController
    drakeQPControllerLcmTranslator
    drakeQPControllerParamParsing
    drakeRigidBodyAliasGroups
    drakeValkyrieConstants)
endif()
