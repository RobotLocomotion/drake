if(lcm_FOUND)
  # Defines a shared library for use by KUKA iiwa demos based on System 2.0.
  add_library_with_exports(LIB_NAME drakeKukaIiwaArmCommon SOURCE_FILES
      iiwa_common.cc iiwa_lcm.cc)
  target_link_libraries(drakeKukaIiwaArmCommon
    drakeLCMSystem2
    drakeRBM)
  drake_install_libraries(drakeKukaIiwaArmCommon)
  drake_install_headers(
      iiwa_common.h iiwa_lcm.h)
  drake_install_pkg_config_file(drake-kuka-iiwa-arm-common
      TARGET drakeKukaIiwaArmCommon
      LIBS -ldrakeKukaIiwaArmCommon
      REQUIRES drake-lcm-system drake-lcmtypes-cpp drake-rbm)

  add_executable(run_kuka_iiwa_arm_dynamics run_kuka_iiwa_arm_dynamics.cc)
  target_link_libraries(run_kuka_iiwa_arm_dynamics
      drakeKukaIiwaArmCommon
      drakeMultibodyParsers
      drakeRigidBodyPlant
      drakeSystemAnalysis
      drakeSystemControllers
      drakeLCMSystem2
      gflags)
  drake_add_test(
      NAME run_kuka_iiwa_arm_dynamics
      COMMAND run_kuka_iiwa_arm_dynamics --duration 0.5)

  add_executable(kuka_ik_demo kuka_ik_demo.cc)
  target_link_libraries(kuka_ik_demo
    drakeIK
    drakeMultibodyParsers
    drakeRBM
    robotlocomotion-lcmtypes-cpp
    lcm)

  add_executable(kuka_plan_runner kuka_plan_runner.cc)
  target_link_libraries(kuka_plan_runner
    drakeKukaIiwaArmCommon
    drakeTrajectories
    drakeMultibodyParsers
    drakeRBM
    drakeLCMTypes
    robotlocomotion-lcmtypes-cpp
    lcm)

  add_executable(kuka_simulation kuka_simulation.cc)
  target_link_libraries(kuka_simulation
    drakeKukaIiwaArmCommon
    drakeMultibodyParsers
    drakeRigidBodyPlant
    drakeSystemAnalysis
    drakeSystemControllers
    drakeLCMSystem2)
  drake_add_test(
      NAME kuka_simulation COMMAND kuka_simulation -simulation_sec 0.1)

endif()

add_subdirectory(controlled_kuka)
add_subdirectory(iiwa_world)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
