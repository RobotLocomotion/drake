if(lcm_FOUND)
  drake_install_headers(Pendulum.h)

  add_library_with_exports(LIB_NAME drakePendulumPlant
    SOURCE_FILES pendulum_swing_up.cc pendulum_plant.cc)
  target_link_libraries(drakePendulumPlant drakeSystemFramework
    drakeDynamicConstraint
    drakeTrajectoryOptimization)
  pods_install_libraries(drakePendulumPlant)
  drake_install_headers(pendulum_plant.h pendulum_swing_up.h)
  pods_install_pkg_config_file(drake-pendulum-system
    LIBS -ldrakePendulumPlant
    REQUIRES
    VERSION 0.0.1)

  add_executable(pendulum_run_dynamics pendulum_run_dynamics.cc)
  target_link_libraries(pendulum_run_dynamics
    drakePendulumPlant
    drakeRigidBodyPlant
    drakeSystemAnalysis)
  drake_add_test(NAME pendulum_run_dynamics
    COMMAND pendulum_run_dynamics)

  add_executable(pendulum_run_energy_shaping pendulum_run_energy_shaping.cc)
  target_link_libraries(pendulum_run_energy_shaping
    drakePendulumPlant
    drakeRigidBodyPlant
    drakeSystemAnalysis)
  drake_add_test(NAME pendulum_run_energy_shaping
    COMMAND pendulum_run_energy_shaping)

  add_executable(pendulum_run_lqr pendulum_run_lqr.cc)
  target_link_libraries(pendulum_run_lqr drakeRBM drakeLCMSystem)
  drake_add_test(NAME pendulum_run_lqr
    COMMAND pendulum_run_lqr --non-realtime)

  add_executable(pendulum_run_swing_up pendulum_run_swing_up.cc)
  target_link_libraries(pendulum_run_swing_up
    drakePendulumPlant
    drakeRigidBodyPlant
    drakeSystemControllers
    drakeSystemAnalysis)
  # TODO(sam.creasey) There is currently no non-linear solver on Windows: see
  # issues #2352, #2569.
  drake_find_package(snopt_c CONFIG)
  drake_find_package(NLopt CONFIG)

  if(ipopt_FOUND OR NLopt_FOUND OR snopt_c_FOUND)
    drake_add_test(NAME pendulum_run_swing_up
      COMMAND pendulum_run_swing_up --non-realtime)
  endif()
endif()

add_subdirectory(test)

drake_add_matlab_test(NAME examples/Pendulum/PendulumEnergyShaping.run OPTIONAL lcm COMMAND PendulumEnergyShaping.run SIZE large)
drake_add_matlab_test(NAME examples/Pendulum/globalStability REQUIRES mosek spotless OPTIONAL lcm COMMAND globalStability)
drake_add_matlab_test(NAME examples/Pendulum/runDLQR OPTIONAL lcm COMMAND runDLQR SIZE large)
drake_add_matlab_test(NAME examples/Pendulum/runLQR OPTIONAL lcm COMMAND runLQR)
drake_add_matlab_test(NAME examples/Pendulum/runLQRtest REQUIRES spotless OPTIONAL lcm mosek sedumi COMMAND runLQRtest)
drake_add_matlab_test(NAME examples/Pendulum/runPassive OPTIONAL lcm COMMAND runPassive)
drake_add_matlab_test(NAME examples/Pendulum/runPassiveURDF OPTIONAL bullet COMMAND runPassiveURDF)
drake_add_matlab_test(NAME examples/Pendulum/runSwingUp OPTIONAL lcm snopt COMMAND runSwingUp)
drake_add_matlab_test(NAME examples/Pendulum/runTrigLQR REQUIRES spotless OPTIONAL lcm COMMAND runTrigLQR)
drake_add_matlab_test(NAME examples/Pendulum/PendulumEnergyControl.run REQUIRES spotless OPTIONAL lcm mosek sedumi COMMAND PendulumEnergyControl.run)

# drake_add_matlab_test(NAME examples/Pendulum/runTrajectorySwingUpAndBalance REQUIRES spotless OPTIONAL mosek sedumi snopt COMMAND runTrajectorySwingUpAndBalance)  # FIXME: see #2305
drake_add_matlab_test(NAME examples/Pendulum/runValueIteration COMMAND runValueIteration SIZE large)
