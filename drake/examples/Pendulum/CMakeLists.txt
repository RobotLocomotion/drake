
include_directories(${PROJECT_SOURCE_DIR}/core)
include_directories(${PROJECT_SOURCE_DIR}/systems)
include_directories(${PROJECT_SOURCE_DIR}/systems/frames)
include_directories(${PROJECT_SOURCE_DIR}/systems/plants)
include_directories(${PROJECT_SOURCE_DIR}/systems/plants/joints)
include_directories(${PROJECT_SOURCE_DIR}/util)

if (LCM_FOUND AND NOT WIN32)  # win32 won't be hard... just have to add declspecs
  include_directories(${PROJECT_SOURCE_DIR}/pod-build/lcmgen)
  add_executable(runPendulumDynamics runDynamics.cpp)
  target_link_libraries(runPendulumDynamics drakeRBM)
  add_dependencies(runPendulumDynamics drake_lcmtypes lcmtype_agg_hpp)
  pods_use_pkg_config_packages(runPendulumDynamics lcm)
  add_test(NAME runPendulumDynamics COMMAND runPendulumDynamics WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/examples/Pendulum)

  add_executable(runPendulumEnergyShaping runEnergyShaping.cpp)
  target_link_libraries(runPendulumEnergyShaping drakeRBM)
  add_dependencies(runPendulumEnergyShaping drake_lcmtypes lcmtype_agg_hpp)
  pods_use_pkg_config_packages(runPendulumEnergyShaping lcm)
  add_test(NAME runPendulumEnergyShaping COMMAND runPendulumEnergyShaping)

  add_executable(runPendulumLQR runLQR.cpp)
  target_link_libraries(runPendulumLQR drakeRBM)
  add_dependencies(runPendulumLQR drake_lcmtypes lcmtype_agg_hpp)
  pods_use_pkg_config_packages(runPendulumLQR lcm)
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
#      NAME "viewer" COMMAND "${CMAKE_INSTALL_PREFIX}/bin/ddConsoleApp -m ddapp.drakevisualizer"
#  )
endif()
