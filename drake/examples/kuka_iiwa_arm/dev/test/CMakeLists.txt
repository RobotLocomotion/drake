if(lcm_FOUND)

  # Currently only SNOPT seems stable to run this demo.
  # NLopt isn't sufficient to run the kuka_demo, as it is unable to
  # find a solution.  See
  # https://github.com/RobotLocomotion/drake/issues/3710
  # Ipopt is currently being investigated for issues as well. See
  # https://github.com/RobotLocomotion/drake/issues/3128.
  # This change was necessitated by build failures reported in :
  # https://github.com/RobotLocomotion/drake/issues/6026
  if(snopt_FOUND)
    drake_add_cc_test(NAME iiwa_ik_test SIZE medium)
    target_link_libraries(iiwa_ik_test drakeKukaIiwaArmIK)
  endif()
endif()
