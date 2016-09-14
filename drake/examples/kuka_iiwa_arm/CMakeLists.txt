if(lcm_FOUND)
  add_executable(run_kuka_iiwa_arm_dynamics run_kuka_iiwa_arm_dynamics.cc)
  target_link_libraries(run_kuka_iiwa_arm_dynamics
    drakeKukaIiwaArm
    drakeRBSystem
    drakeLCMSystem)
  drake_add_test(NAME run_kuka_iiwa_arm_dynamics COMMAND run_kuka_iiwa_arm_dynamics --duration 0.5)

  add_library_with_exports(LIB_NAME drakeKukaIiwaArm SOURCE_FILES
          iiwa_simulation.cc)
  target_link_libraries(drakeKukaIiwaArm
    drakeRBSystem
    drakeShapes
    drakeCommon
    drakeLCMSystem)
  pods_install_libraries(drakeKukaIiwaArm)
  drake_install_headers(
          iiwa_simulation.h)
  pods_install_pkg_config_file(drake-kuka-iiwa-arm
          LIBS -ldrakeKukaIiwaArm
          REQUIRES
          VERSION 0.0.1)
  add_dependencies(drakeKukaIiwaArm
    drake_lcmtypes_hpp)

  add_executable(kuka_lcm_visualizer kuka_lcm_visualizer.cc)
  target_link_libraries(kuka_lcm_visualizer
    drakeKukaIiwaArm
    drake_lcmtypes
    threads)
  add_executable(kuka_ik_demo kuka_ik_demo.cc)
  target_link_libraries(kuka_ik_demo drakeIK
    drakeTrajectories
    drakeRBM
    drakeLCMSystem)

  add_executable(run_kuka_iiwa_gravity_compensated_torque_control
          run_kuka_iiwa_gravity_compensated_torque_control.cc)
  target_link_libraries(run_kuka_iiwa_gravity_compensated_torque_control
          drakeKukaIiwaArm
          drakeRBSystem
          drakeLCMSystem
          gflags)

  add_executable(run_kuka_iiwa_gravity_compensated_position_control
          run_kuka_iiwa_gravity_compensated_position_control.cc)
  target_link_libraries(run_kuka_iiwa_gravity_compensated_position_control
          drakeKukaIiwaArm
          drakeRBSystem
          drakeLCMSystem
          gflags)
endif()

add_subdirectory(test)
