if(lcm_FOUND)
  drake_add_cc_test(robot_state_encoder_decoder_test)
  add_definitions(-g)
  target_link_libraries(robot_state_encoder_decoder_test
    drakeMultibodyParsers
    drakeRobotStateDecoder
    drakeRobotStateEncoder
    drakeSystemPrimitives)

  drake_add_cc_test(robot_command_to_desired_effort_converter_test)
  target_link_libraries(robot_command_to_desired_effort_converter_test
    drakeMultibodyParsers
    drakeRBM
    drakeSystemPrimitives
    drakeRobotCommandToDesiredEffortConverter)

  drake_add_cc_test(NAME valkyrie_ik_test
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}")
  target_link_libraries(valkyrie_ik_test
    drakeCommon
    drakeIK
    drakeLcm
    drakeMultibodyParsers
    drakeRBM
    drakeRigidBodyPlant)

  drake_add_cc_test(valkyrie_simulation_test)
  target_link_libraries(valkyrie_simulation_test
      drakeLcmSystem
      drakeMultibodyParsers
      drakeRigidBodyPlant
      drakeRobotStateEncoder
      drakeSystemAnalysis
      drakeSystemFramework
      drakeSystemPrimitives
      drakeActuatorEffortToRigidBodyPlantInputConverter
      drakeRobotCommandToDesiredEffortConverter
      drakeValkyrieConstants)
endif()
