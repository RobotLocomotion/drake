if(lcm_FOUND)
  drake_add_cc_test(pendulum_urdf_dynamics_test)
  target_link_libraries(pendulum_urdf_dynamics_test
      drakePendulumPlant
      drakeRigidBodyPlant)

  drake_add_cc_test(pendulum_dynamic_constraint_test)
  target_link_libraries(pendulum_dynamic_constraint_test
    drakePendulumPlant drakeTrajectoryOptimization)

  drake_add_cc_test(pendulum_plant_test)
  target_link_libraries(pendulum_plant_test drakePendulumPlant)

  if(ipopt_FOUND OR NLopt_FOUND OR snopt_FOUND)
    drake_add_cc_test(pendulum_trajectory_optimization_test)
    target_link_libraries(pendulum_trajectory_optimization_test
      drakePendulumPlant drakeTrajectoryOptimization)
  endif()
endif()
