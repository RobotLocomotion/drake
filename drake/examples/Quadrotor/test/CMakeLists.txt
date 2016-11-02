if(lcm_FOUND)
  drake_add_cc_test(quadrotor_urdf_dynamics_test)
  target_link_libraries(quadrotor_urdf_dynamics_test drakeRBSystem)
endif()

# drake_add_matlab_test(NAME examples/Quadrotor/test/buildOcTree COMMAND buildOcTree)  # FIXME: see #2100
drake_add_matlab_test(NAME examples/Quadrotor/test/manipulatorGradients OPTIONAL bullet COMMAND manipulatorGradients)
drake_add_matlab_test(NAME examples/Quadrotor/test/propellorGradients OPTIONAL bullet COMMAND propellorGradients)
drake_add_matlab_test(NAME examples/Quadrotor/test/quadrotorDynamics OPTIONAL bullet COMMAND quadrotorDynamics)
drake_add_matlab_test(NAME examples/Quadrotor/test/testCollisions OPTIONAL bullet gurobi snopt COMMAND testCollisions)
drake_add_matlab_test(NAME examples/Quadrotor/test/testInvertFlatOutputs OPTIONAL bullet snopt COMMAND testInvertFlatOutputs)
drake_add_matlab_test(NAME examples/Quadrotor/test/testMISOSTrajectory REQUIRES gurobi yalmip OPTIONAL iris COMMAND testMISOSTrajectory)
