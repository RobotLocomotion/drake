if(lcm_FOUND)
  add_executable(controlled_kuka_demo
      controlled_kuka_demo.cc)
  target_link_libraries(controlled_kuka_demo
      drakeKukaIiwaArmCommon
      drakeMultibodyParsers
      drakeRigidBodyPlant
      drakeSystemAnalysis
      drakeSystemControllers
      drakeLCMSystem2
      drakeTrajectories
      gflags)

  add_executable(kuka_cartesian_way_point_demo
      kuka_cartesian_way_point_demo.cc)
  target_link_libraries(kuka_cartesian_way_point_demo
      drakeKukaIiwaArmCommon
      drakeRigidBodyPlant
      drakeSystemAnalysis
      drakeSystemControllers
      drakeLCMSystem2
      drakeTrajectories
      gflags)

  # NLopt isn't sufficient to run the kuka_demo, as it is unable to
  # find a solution.  See
  # https://github.com/RobotLocomotion/drake/issues/3710
  if(snopt_FOUND OR ipopt_FOUND)
    drake_add_test(NAME kuka_demo COMMAND kuka_demo)
  endif()
endif()
