if(lcm_FOUND)
  add_executable(controlled_kuka_demo
      controlled_kuka_demo.cc)
  target_link_libraries(controlled_kuka_demo
      drakeKukaIiwaArmCommon
      drakeLcmSystem
      drakeMultibodyParsers
      drakeRigidBodyPlant
      drakeSimDiagramBuilder
      drakeSystemAnalysis
      drakeSystemControllers
      drakeTrajectories
      gflags)

  add_executable(kuka_cartesian_way_point_demo
      kuka_cartesian_way_point_demo.cc)
  target_link_libraries(kuka_cartesian_way_point_demo
      drakeKukaIiwaArmCommon
      drakeLcmSystem
      drakeRigidBodyPlant
      drakeSimDiagramBuilder
      drakeSystemAnalysis
      drakeSystemControllers
      drakeTrajectories
      gflags)

  # NLopt isn't sufficient to run the kuka_demo, as it is unable to
  # find a solution.  See
  # https://github.com/RobotLocomotion/drake/issues/3710
  # Ipopt is currently being investigated for issues as well. See
  # https://github.com/RobotLocomotion/drake/issues/3128
  if(snopt_FOUND)
    drake_add_test(NAME controlled_kuka_demo
        COMMAND controlled_kuka_demo)
    drake_add_test(NAME kuka_cartesian_way_point_demo
        COMMAND kuka_cartesian_way_point_demo)
  endif()
endif()
