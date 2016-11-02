if(lcm_FOUND)
  # Defines a shared library for use by KUKA iiwa demos based on System 2.0.
  add_library_with_exports(LIB_NAME drakeKukaIiwaArmCommon SOURCE_FILES
      iiwa_common.cc)
  target_link_libraries(drakeKukaIiwaArmCommon
      drakeRBM)
  pods_install_libraries(drakeKukaIiwaArmCommon)
  drake_install_headers(
      iiwa_common.h)
  pods_install_pkg_config_file(drake-kuka-iiwa-arm-common
      LIBS -ldrakeKukaIiwaArmCommon
      REQUIRES
      VERSION 0.0.1)

  # TODO(liang.fok) Remove this library once all System 1.0-based KUKA iiwa
  # demos that are removed.
  add_library_with_exports(LIB_NAME drakeKukaIiwaArm SOURCE_FILES
      iiwa_simulation.cc)
  target_link_libraries(drakeKukaIiwaArm
      drakeRBSystem
      drakeShapes
      drakeCommon
      drakeLCMSystem
      drakeLCMTypes)
  pods_install_libraries(drakeKukaIiwaArm)
  drake_install_headers(
      iiwa_simulation.h)
  pods_install_pkg_config_file(drake-kuka-iiwa-arm
      LIBS -ldrakeKukaIiwaArm
      REQUIRES
      VERSION 0.0.1)

  add_executable(run_kuka_iiwa_arm_dynamics run_kuka_iiwa_arm_dynamics.cc)
  target_link_libraries(run_kuka_iiwa_arm_dynamics
      drakeKukaIiwaArmCommon
      drakeRigidBodyPlant
      drakeSystemAnalysis
      drakeSystemControllers
      drakeLCMSystem2
      gflags)
  drake_add_test(
      NAME run_kuka_iiwa_arm_dynamics
      COMMAND run_kuka_iiwa_arm_dynamics --duration 0.5)

  add_executable(kuka_lcm_visualizer kuka_lcm_visualizer.cc)
  target_link_libraries(kuka_lcm_visualizer
      drakeKukaIiwaArm
      drake_lcmtypes
      Threads::Threads)

  add_executable(kuka_ik_demo kuka_ik_demo.cc)
  target_link_libraries(kuka_ik_demo
    drakeIK
    drakeRBM
    robotlocomotion-lcmtypes-cpp
    lcm)

  add_executable(kuka_plan_runner kuka_plan_runner.cc)
  target_link_libraries(kuka_plan_runner
    drakeKukaIiwaArmCommon
    drakeTrajectories
    drakeRBM
    drakeLCMTypes
    robotlocomotion-lcmtypes-cpp
    lcm)

  add_executable(kuka_simulation kuka_simulation.cc)
  target_link_libraries(kuka_simulation
    drakeKukaIiwaArmCommon
    drakeRigidBodyPlant
    drakeSystemAnalysis
    drakeSystemControllers
    drakeLCMSystem2)
  drake_add_test(
      NAME kuka_simulation COMMAND kuka_simulation -simulation_sec 0.1)

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

add_subdirectory(controlled_kuka)
add_subdirectory(test)
