if(lcm_FOUND)
  add_executable(kuka_demo kuka_demo.cc)
  target_link_libraries(kuka_demo
      drakeIK
      drakeKukaIiwaArmCommon
      drakeRigidBodyPlant
      drakeSystemAnalysis
      drakeSystemControllers
      drakeLCMSystem2
      drakeTrajectories
      GTest::GTest GTest::Main)
  # NLopt isn't sufficient to run the kuka_demo, as it is unable to
  # find a solution.  See
  # https://github.com/RobotLocomotion/drake/issues/3710
  if(snopt_c_FOUND OR ipopt_FOUND)
    add_test(NAME kuka_demo COMMAND kuka_demo)
  endif()
endif()
