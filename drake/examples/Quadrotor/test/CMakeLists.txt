if(LCM_FOUND AND GTEST_FOUND)
  add_executable(quadrotorURDFDynamicsTest urdfDynamicsTest.cpp)
  target_link_libraries(quadrotorURDFDynamicsTest drakeRBSystem ${GTEST_BOTH_LIBRARIES})
  add_test(NAME quadrotorURDFDynamicsTest COMMAND quadrotorURDFDynamicsTest)
endif()

# add_matlab_test(NAME examples/Quadrotor/test/buildOcTree COMMAND buildOcTree)  # FIXME: see #2100
add_matlab_test(NAME examples/Quadrotor/test/manipulatorGradients COMMAND manipulatorGradients)
add_matlab_test(NAME examples/Quadrotor/test/propellorGradients COMMAND propellorGradients)
add_matlab_test(NAME examples/Quadrotor/test/quadrotorDynamics COMMAND quadrotorDynamics)
add_matlab_test(NAME examples/Quadrotor/test/testCollisions COMMAND testCollisions)
add_matlab_test(NAME examples/Quadrotor/test/testInvertFlatOutputs COMMAND testInvertFlatOutputs)
add_matlab_test(NAME examples/Quadrotor/test/testMISOSTrajectory COMMAND testMISOSTrajectory)
