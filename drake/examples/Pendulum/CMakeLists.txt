if (LCM_FOUND)
  add_executable(runPendulumDynamics runDynamics.cpp)
  target_link_libraries(runPendulumDynamics drakeRBM drakeLCMSystem)
  add_test(NAME runPendulumDynamics COMMAND runPendulumDynamics WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/examples/Pendulum)

  add_executable(runPendulumEnergyShaping runEnergyShaping.cpp)
  target_link_libraries(runPendulumEnergyShaping drakeRBM drakeLCMSystem)
#  add_test(NAME runPendulumEnergyShaping COMMAND runPendulumEnergyShaping)
#  set_tests_properties(runPendulumEnergyShaping PROPERTIES TIMEOUT 10)
    # disabled for now since it's not a good test, since it will always timeout

  add_executable(runPendulumLQR runLQR.cpp)
  target_link_libraries(runPendulumLQR drakeRBM drakeLCMSystem)
  add_test(NAME runPendulumLQR COMMAND runPendulumLQR WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/examples/Pendulum)

#  write_procman(Pendulum.pmd
#      GROUP "Simulate"
#        NAME "dynamics (cpp)" COMMAND "bash -c 'cd ${CMAKE_CURRENT_SOURCE_DIR} && ${CMAKE_BINARY_DIR}/bin/runPendulumDynamics'"
#      ENDGROUP
#      GROUP "LCM Controllers"
#        NAME "energy shaping (cpp)" COMMAND "${CMAKE_BINARY_DIR}/bin/runPendulumEnergyShaping"
#      ENDGROUP
#      GROUP "Controllers+Simulation In One"
#        NAME "LQR (cpp)" COMMAND "bash -c 'cd ${CMAKE_CURRENT_SOURCE_DIR} && ${CMAKE_BINARY_DIR}/bin/runPendulumLQR'"
#      ENDGROUP
#      NAME "viewer" COMMAND "${CMAKE_INSTALL_PREFIX}/bin/drake-visualizer"
#  )
endif()

add_subdirectory(test)
