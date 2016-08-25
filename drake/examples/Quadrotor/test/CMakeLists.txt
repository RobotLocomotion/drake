if(lcm_FOUND)
  add_executable(quadrotorURDFDynamicsTest urdfDynamicsTest.cpp)
  target_link_libraries(quadrotorURDFDynamicsTest drakeRBSystem ${GTEST_BOTH_LIBRARIES})
  add_test(NAME quadrotorURDFDynamicsTest COMMAND quadrotorURDFDynamicsTest)
endif()

# add_matlab_test(NAME examples/Quadrotor/test/buildOcTree COMMAND buildOcTree)  # FIXME: see #2100
add_matlab_test(NAME examples/Quadrotor/test/manipulatorGradients OPTIONAL bullet COMMAND manipulatorGradients)
add_matlab_test(NAME examples/Quadrotor/test/propellorGradients OPTIONAL bullet COMMAND propellorGradients)
add_matlab_test(NAME examples/Quadrotor/test/quadrotorDynamics OPTIONAL bullet COMMAND quadrotorDynamics)
add_matlab_test(NAME examples/Quadrotor/test/testCollisions OPTIONAL bullet gurobi snopt COMMAND testCollisions)
add_matlab_test(NAME examples/Quadrotor/test/testInvertFlatOutputs OPTIONAL bullet snopt COMMAND testInvertFlatOutputs)
add_matlab_test(NAME examples/Quadrotor/test/testMISOSTrajectory REQUIRES gurobi yalmip OPTIONAL iris COMMAND testMISOSTrajectory)
