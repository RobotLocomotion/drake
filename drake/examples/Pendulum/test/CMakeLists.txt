if(LCM_FOUND AND GTEST_FOUND)
  add_executable(pendulumURDFDynamicsTest urdfDynamicsTest.cpp)
  target_link_libraries(pendulumURDFDynamicsTest drakeRBSystem ${GTEST_BOTH_LIBRARIES})
  add_test(NAME pendulumURDFDynamicsTest COMMAND pendulumURDFDynamicsTest)
endif()

add_matlab_test(NAME examples/Pendulum/test/coordinateTest COMMAND coordinateTest)
add_matlab_test(NAME examples/Pendulum/test/dynamicsGradientsTest COMMAND dynamicsGradientsTest)
add_matlab_test(NAME examples/Pendulum/test/polyFeedback COMMAND polyFeedback)
add_matlab_test(NAME examples/Pendulum/test/runPendLCMControl COMMAND runPendLCMControl)
add_matlab_test(NAME examples/Pendulum/test/testFeedback COMMAND testFeedback)
add_matlab_test(NAME examples/Pendulum/test/testFrictionModel COMMAND testFrictionModel)
# add_matlab_test(NAME examples/Pendulum/test/testLCMPlant COMMAND testLCMPlant)  # FIXME: see #310
# set_tests_properties(examples/Pendulum/test/testLCMPlant PROPERTIES RUN_SERIAL)  # asked kitware about this
add_matlab_test(NAME examples/Pendulum/test/trigPolyTest COMMAND trigPolyTest)
add_matlab_test(NAME examples/Pendulum/test/tvlqrRestartTest COMMAND tvlqrRestartTest)
add_matlab_test(NAME examples/Pendulum/test/tvlqrTest COMMAND tvlqrTest)
add_matlab_test(NAME examples/Pendulum/test/urdfDynamicsTest COMMAND urdfDynamicsTest)
