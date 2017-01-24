if(gurobi_FOUND AND lcm_FOUND AND yaml-cpp_FOUND)
  add_executable(valkyrie_balancing_controller_system
    atlas_joint_level_controller_system.cc
    joint_level_controller_system.cc
    plan_eval_system.cc
    humanoid_plan_eval_system.cc
    qp_controller_system.cc
    robot_state_decoder_system.cc
    valkyrie_balancing_controller_system.cc)
  target_link_libraries(valkyrie_balancing_controller_system
    drakeLCMSystem2
    drakeMultibodyParsers
    drakeQPController
    drakeQPControllerLcmTranslator
    drakeQPControllerParamParsing
    drakeRigidBodyAliasGroups
    drakeValkyrieConstants)
endif()

if(gurobi_FOUND AND yaml-cpp_FOUND)
  add_library(kuka_traj_follower_system
    kuka_plan_eval_system.cc
    plan_eval_system.cc
    qp_controller_system.cc
    robot_status_wrapper.cc
    joint_level_controller_system.cc)
  target_link_libraries(kuka_traj_follower_system
    drakeMultibodyParsers
    drakeQPController
    drakeQPControllerParamParsing
    drakeRigidBodyAliasGroups
    drakeSystemFramework
    drakeTrajectories)
endif()
