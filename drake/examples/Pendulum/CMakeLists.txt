
include_directories(${PROJECT_SOURCE_DIR}/systems)
include_directories(${PROJECT_SOURCE_DIR}/systems/frames)
include_directories(${PROJECT_SOURCE_DIR}/systems/plants)
include_directories(${PROJECT_SOURCE_DIR}/systems/plants/joints)
include_directories(${PROJECT_SOURCE_DIR}/util)

if (LCM_FOUND)
  include_directories(${PROJECT_SOURCE_DIR}/pod-build/lcmgen)
  add_executable(runPendulumDynamics runDynamics.cpp)
  target_link_libraries(runPendulumDynamics drakeSystem drakeRBM drakeGeometryUtil)
  add_test(NAME runPendulumDynamics COMMAND runPendulumDynamics WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/examples/Pendulum)

  add_executable(runPendulumEnergyShaping runEnergyShaping.cpp)
  target_link_libraries(runPendulumEnergyShaping drakeSystem)
  add_test(NAME runPendulumEnergyShaping COMMAND runPendulumEnergyShaping)

  add_executable(runPendulumLQR runLQR.cpp)
  target_link_libraries(runPendulumLQR drakeSystem)
  add_test(NAME runPendulumLQR COMMAND runPendulumLQR)

  write_procman(Pendulum.pmd
      GROUP "Simulate"
        NAME "cpp dynamics" COMMAND "bash -c 'cd ${CMAKE_CURRENT_SOURCE_DIR} && ${CMAKE_BINARY_DIR}/bin/runPendulumDynamics'"
      ENDGROUP
      NAME "swingup controller" COMMAND "${CMAKE_BINARY_DIR}/bin/runPendulumEnergyShaping"
      NAME "director" COMMAND "${CMAKE_INSTALL_PREFIX}/bin/ddConsoleApp -m ddapp.drakevisualizer"
  )
endif()