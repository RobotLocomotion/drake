if(lcm_FOUND)
  add_library_with_exports(LIB_NAME drakePendulumPlant
    SOURCE_FILES pendulum_swing_up.cc pendulum_plant.cc)
  target_link_libraries(drakePendulumPlant
    drakeMultibodyParsers
    drakeSystemFramework
    drakeTrajectoryOptimization
    Eigen3::Eigen)
  drake_install_libraries(drakePendulumPlant)
  drake_install_headers(pendulum_plant.h pendulum_swing_up.h)
  drake_install_pkg_config_file(drake-pendulum-plant
    TARGET drakePendulumPlant
    LIBS -ldrakePendulumPlant
    REQUIRES
      drake-trajectory-optimization
      eigen3)

  add_executable(pendulum_run_dynamics pendulum_run_dynamics.cc)
  target_link_libraries(pendulum_run_dynamics
    drakeMultibodyParsers
    drakePendulumPlant
    drakeRigidBodyPlant
    drakeSystemAnalysis
    drakeSystemPrimitives)
  drake_add_test(NAME pendulum_run_dynamics
    COMMAND pendulum_run_dynamics)

  add_executable(pendulum_run_energy_shaping pendulum_run_energy_shaping.cc)
  target_link_libraries(pendulum_run_energy_shaping
    drakeMultibodyParsers
    drakePendulumPlant
    drakeRigidBodyPlant
    drakeSystemAnalysis)
  drake_add_test(NAME pendulum_run_energy_shaping
    COMMAND pendulum_run_energy_shaping)

  add_executable(pendulum_run_lqr pendulum_run_lqr.cc)
  target_link_libraries(pendulum_run_lqr
      drakeMultibodyParsers
      drakePendulumPlant
      drakeRigidBodyPlant
      drakeSystemAnalysis
      drakeLinearControl)
  drake_add_test(NAME pendulum_run_lqr
    COMMAND pendulum_run_lqr)

  add_executable(pendulum_run_swing_up pendulum_run_swing_up.cc)
  target_link_libraries(pendulum_run_swing_up
    drakePendulumPlant
    drakeRigidBodyPlant
    drakeSystemControllers
    drakeSystemAnalysis)

  if(ipopt_FOUND OR NLopt_FOUND OR snopt_FOUND)
    drake_add_test(NAME pendulum_run_swing_up
      COMMAND pendulum_run_swing_up --target_realtime_rate 0.0)
  endif()
endif()

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
