if(lcm_FOUND)
  add_executable(pendulumURDFDynamicsTest urdfDynamicsTest.cpp)
  target_link_libraries(pendulumURDFDynamicsTest drakeRBSystem drakeLCMSystem ${GTEST_BOTH_LIBRARIES})
  drake_add_test(NAME pendulumURDFDynamicsTest COMMAND pendulumURDFDynamicsTest)

  add_executable(pendulum_dynamic_constraint_test pendulum_dynamic_constraint_test.cc)
  target_link_libraries(pendulum_dynamic_constraint_test
    drakeDynamicConstraint drakeLCMSystem ${GTEST_BOTH_LIBRARIES})
  drake_add_test(NAME pendulum_dynamic_constraint_test
    COMMAND pendulum_dynamic_constraint_test)

  add_executable(pendulum_trajectory_optimization_test
    pendulum_trajectory_optimization_test.cc)
  target_link_libraries(pendulum_trajectory_optimization_test
    drakePendulum drakeTrajectoryOptimization ${GTEST_BOTH_LIBRARIES})
  # TODO(sam.creasey) There is currently no non-linear solver on Windows: see
  # issues #2352, #2569.
  find_package(NLopt)
  if(ipopt_FOUND OR NLopt_FOUND OR snopt_c_FOUND)
    drake_add_test(NAME pendulum_trajectory_optimization_test
      COMMAND pendulum_trajectory_optimization_test)
  endif()
endif()

drake_add_matlab_test(NAME examples/Pendulum/test/coordinateTest OPTIONAL bullet COMMAND coordinateTest)
drake_add_matlab_test(NAME examples/Pendulum/test/dynamicsGradientsTest COMMAND dynamicsGradientsTest)
drake_add_matlab_test(NAME examples/Pendulum/test/polyFeedback REQUIRES spotless OPTIONAL lcm COMMAND polyFeedback)
drake_add_matlab_test(NAME examples/Pendulum/test/runPendLCMControl REQUIRES lcm COMMAND runPendLCMControl)
drake_add_matlab_test(NAME examples/Pendulum/test/testFeedback REQUIRES spotless OPTIONAL lcm COMMAND testFeedback)
drake_add_matlab_test(NAME examples/Pendulum/test/testFrictionModel OPTIONAL bullet COMMAND testFrictionModel)
# drake_add_matlab_test(NAME examples/Pendulum/test/testLCMPlant REQUIRES lcm COMMAND testLCMPlant)  # FIXME: see #310
# set_tests_properties(examples/Pendulum/test/testLCMPlant PROPERTIES RUN_SERIAL)  # asked kitware about this
drake_add_matlab_test(NAME examples/Pendulum/test/trigPolyTest REQUIRES spotless OPTIONAL bullet lcm COMMAND trigPolyTest)
drake_add_matlab_test(NAME examples/Pendulum/test/tvlqrRestartTest REQUIRES spotless OPTIONAL lcm snopt COMMAND tvlqrRestartTest)
drake_add_matlab_test(NAME examples/Pendulum/test/tvlqrTest REQUIRES spotless OPTIONAL lcm snopt COMMAND tvlqrTest)
drake_add_matlab_test(NAME examples/Pendulum/test/urdfDynamicsTest OPTIONAL bullet lcm COMMAND urdfDynamicsTest)
