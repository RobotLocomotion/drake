if(lcm_FOUND)
  add_executable(quadrotorURDFDynamicsTest urdfDynamicsTest.cpp)
  target_link_libraries(quadrotorURDFDynamicsTest drakeRBSystem ${GTEST_BOTH_LIBRARIES})
  drake_add_test(NAME quadrotorURDFDynamicsTest COMMAND quadrotorURDFDynamicsTest)
endif()

# drake_add_matlab_test(NAME examples/Quadrotor/test/buildOcTree COMMAND buildOcTree)  # FIXME: see #2100
drake_add_matlab_test(NAME examples/Quadrotor/test/manipulatorGradients OPTIONAL bullet COMMAND manipulatorGradients)
drake_add_matlab_test(NAME examples/Quadrotor/test/propellorGradients OPTIONAL bullet COMMAND propellorGradients)
drake_add_matlab_test(NAME examples/Quadrotor/test/quadrotorDynamics OPTIONAL bullet COMMAND quadrotorDynamics)
drake_add_matlab_test(NAME examples/Quadrotor/test/testCollisions OPTIONAL bullet gurobi snopt COMMAND testCollisions)
drake_add_matlab_test(NAME examples/Quadrotor/test/testInvertFlatOutputs OPTIONAL bullet snopt COMMAND testInvertFlatOutputs)
drake_add_matlab_test(NAME examples/Quadrotor/test/testMISOSTrajectory REQUIRES gurobi yalmip OPTIONAL iris COMMAND testMISOSTrajectory)
