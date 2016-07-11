if(lcm_FOUND)
  add_executable(run_kuka_iiwa_arm_dynamics run_kuka_iiwa_arm_dynamics.cc)
  target_link_libraries(run_kuka_iiwa_arm_dynamics drakeRBSystem drakeLCMSystem)
  add_test(NAME run_kuka_iiwa_arm_dynamics COMMAND run_kuka_iiwa_arm_dynamics --duration 0.5)

  add_executable(kuka_lcm_visualizer kuka_lcm_visualizer.cc)
  target_link_libraries(kuka_lcm_visualizer drakeRBM drakeLCMSystem
    drake_lcmtypes threads)
  add_executable(kuka_ik_demo kuka_ik_demo.cc)
  target_link_libraries(kuka_ik_demo drakeIK drakeRBM drakeLCMSystem)

endif()
