if(lcm_FOUND)
  add_executable(pendulumURDFDynamicsTest urdfDynamicsTest.cpp)
  target_link_libraries(pendulumURDFDynamicsTest drakeRBSystem ${GTEST_BOTH_LIBRARIES})
  add_test(NAME pendulumURDFDynamicsTest COMMAND pendulumURDFDynamicsTest)

  add_executable(pendulum_dynamic_constraint_test pendulum_dynamic_constraint_test.cc)
  target_link_libraries(pendulum_dynamic_constraint_test
    drakeDynamicConstraint ${GTEST_BOTH_LIBRARIES})
  add_test(NAME pendulum_dynamic_constraint_test
    COMMAND pendulum_dynamic_constraint_test)

  add_executable(pendulum_trajectory_optimization_test
    pendulum_trajectory_optimization_test.cc)
  target_link_libraries(pendulum_trajectory_optimization_test
    drakePendulum drakeTrajectoryOptimization ${GTEST_BOTH_LIBRARIES})
  # TODO(sam.creasey) There is currently no non-linear solver on Windows: see
  # issues #2352, #2569.
  if(ipopt_FOUND OR nlopt_FOUND OR snopt_c_FOUND)
    add_test(NAME pendulum_trajectory_optimization_test
      COMMAND pendulum_trajectory_optimization_test)
  endif()
endif()

add_matlab_test(NAME examples/Pendulum/test/coordinateTest OPTIONAL bullet COMMAND coordinateTest)
add_matlab_test(NAME examples/Pendulum/test/dynamicsGradientsTest COMMAND dynamicsGradientsTest)
add_matlab_test(NAME examples/Pendulum/test/polyFeedback REQUIRES spotless OPTIONAL lcm COMMAND polyFeedback)
add_matlab_test(NAME examples/Pendulum/test/runPendLCMControl REQUIRES lcm COMMAND runPendLCMControl)
add_matlab_test(NAME examples/Pendulum/test/testFeedback REQUIRES spotless OPTIONAL lcm COMMAND testFeedback)
add_matlab_test(NAME examples/Pendulum/test/testFrictionModel OPTIONAL bullet COMMAND testFrictionModel)
# add_matlab_test(NAME examples/Pendulum/test/testLCMPlant REQUIRES lcm COMMAND testLCMPlant)  # FIXME: see #310
# set_tests_properties(examples/Pendulum/test/testLCMPlant PROPERTIES RUN_SERIAL)  # asked kitware about this
add_matlab_test(NAME examples/Pendulum/test/trigPolyTest REQUIRES spotless OPTIONAL bullet lcm COMMAND trigPolyTest)
add_matlab_test(NAME examples/Pendulum/test/tvlqrRestartTest REQUIRES spotless OPTIONAL lcm snopt COMMAND tvlqrRestartTest)
add_matlab_test(NAME examples/Pendulum/test/tvlqrTest REQUIRES spotless OPTIONAL lcm snopt COMMAND tvlqrTest)
add_matlab_test(NAME examples/Pendulum/test/urdfDynamicsTest OPTIONAL bullet lcm COMMAND urdfDynamicsTest)
