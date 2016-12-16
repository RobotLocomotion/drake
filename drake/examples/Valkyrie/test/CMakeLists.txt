# Deactivated due to memory flakiness in instantaneousQP; see #2165, #2376.
# drake_add_matlab_test(NAME examples/Valkyrie/test/runValkyrieFastWalking REQUIRES gurobi lcm COMMAND runValkyrieFastWalking)
# drake_add_matlab_test(NAME examples/Valkyrie/test/runValkyrieTurning REQUIRES gurobi lcm COMMAND runValkyrieTurning)
# drake_add_matlab_test(NAME examples/Valkyrie/test/runValkyrieTurning270 REQUIRES gurobi lcm COMMAND runValkyrieTurning270)

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

  add_executable(valkyrie_ik_test valkyrie_ik_test.cc)
  target_link_libraries(valkyrie_ik_test
    GTest::GTest GTest::Main
    drakeCommon
    drakeIK
    drakeLcm
    drakeMultibodyParsers
    drakeRBM
    drakeRigidBodyPlant
    drakeSystemFramework
    drakeSystemAnalysis
    drakeSystemFramework
    drakeSystemPrimitives)
  drake_add_test(NAME valkyrie_ik_test WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}" COMMAND valkyrie_ik_test)
endif()
