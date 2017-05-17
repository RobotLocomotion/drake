# NLopt isn't sufficient to run the kuka_demo, as it is unable to
# find a solution.  See
# https://github.com/RobotLocomotion/drake/issues/3710
# This change was necessitated by build failures reported in :
# https://github.com/RobotLocomotion/drake/issues/6026
if (snopt_FOUND OR ipopt_FOUND)
  drake_add_cc_test(NAME relaxing_ik_wrapper_test SIZE medium)
  target_link_libraries(relaxing_ik_wrapper_test drakeManipulationPlanner)
endif()
