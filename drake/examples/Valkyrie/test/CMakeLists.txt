# Deactivated due to memory flakiness in instantaneousQP; see #2165, #2376.
# drake_add_matlab_test(NAME examples/Valkyrie/test/runValkyrieFastWalking REQUIRES gurobi lcm COMMAND runValkyrieFastWalking)
# drake_add_matlab_test(NAME examples/Valkyrie/test/runValkyrieTurning REQUIRES gurobi lcm COMMAND runValkyrieTurning)
# drake_add_matlab_test(NAME examples/Valkyrie/test/runValkyrieTurning270 REQUIRES gurobi lcm COMMAND runValkyrieTurning270)

if(lcm_FOUND)
  drake_add_cc_test(robot_state_encoder_decoder_test)
  target_link_libraries(robot_state_encoder_decoder_test
      drakeRobotStateEncoder
      drakeRobotStateDecoder)
endif()