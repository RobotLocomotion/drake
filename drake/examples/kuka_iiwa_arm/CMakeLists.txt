if(lcm_FOUND)
  add_library_with_exports(LIB_NAME drakeSimDiagramBuilder SOURCE_FILES
      sim_diagram_builder.cc)
  target_link_libraries(drakeSimDiagramBuilder
      drakeRigidBodyPlant
      drakeSystemAnalysis
      drakeSystemControllers
      drakeLcmSystem)
  drake_install_libraries(drakeSimDiagramBuilder)
  drake_install_headers(
      sim_diagram_builder.h)
  drake_install_pkg_config_file(drake-sim-diagram-builder
      TARGET drakeSimDiagramBuilder
      LIBS -ldrakeSimDiagramBuilder
      REQUIRES drake-lcm-system drake-lcmtypes-cpp drake-rbm eigen3)

  # Defines a shared library for use by KUKA iiwa demos based on System 2.0.
  add_library_with_exports(LIB_NAME drakeKukaIiwaArmCommon SOURCE_FILES
      iiwa_common.cc iiwa_lcm.cc iiwa_plan_source.cc)
  target_link_libraries(drakeKukaIiwaArmCommon
      drakeIK
      drakeLcmSystem
      drakeRBM
      drakeMultibodyParsers
      robotlocomotion-lcmtypes-cpp
      Eigen3::Eigen)
  drake_install_libraries(drakeKukaIiwaArmCommon)
  drake_install_headers(
      iiwa_common.h iiwa_lcm.h iiwa_plan_source.h)
  drake_install_pkg_config_file(drake-kuka-iiwa-arm-common
      TARGET drakeKukaIiwaArmCommon
      LIBS -ldrakeKukaIiwaArmCommon
      REQUIRES drake-lcm-system drake-lcmtypes-cpp drake-rbm eigen3)

  add_executable(run_kuka_iiwa_arm_dynamics run_kuka_iiwa_arm_dynamics.cc)
  target_link_libraries(run_kuka_iiwa_arm_dynamics
      drakeSimDiagramBuilder
      drakeKukaIiwaArmCommon
      drakeLcmSystem
      drakeMultibodyParsers
      drakeRigidBodyPlant
      drakeSystemAnalysis
      drakeSystemControllers
      gflags)
  drake_add_test(
      NAME run_kuka_iiwa_arm_dynamics
      COMMAND run_kuka_iiwa_arm_dynamics --duration 0.5
      SIZE medium)

  add_executable(kuka_ik_demo kuka_ik_demo.cc)
  target_link_libraries(kuka_ik_demo
    drakeIK
    drakeKukaIiwaArmCommon
    drakeMultibodyParsers
    drakeRBM
    robotlocomotion-lcmtypes-cpp
    lcm)

  add_executable(kuka_plan_runner kuka_plan_runner.cc)
  target_link_libraries(kuka_plan_runner
    drakeKukaIiwaArmCommon
    drakeLCMTypes
    drakeTrajectories
    drakeMultibodyParsers
    drakeRBM
    robotlocomotion-lcmtypes-cpp
    lcm)

  add_executable(iiwa_controller iiwa_controller.cc)
  target_link_libraries(iiwa_controller
    drakeKukaIiwaArmCommon
    drakeLCMTypes
    drakeTrajectories
    drakeMultibodyParsers
    drakeLcmSystem
    drakeRBM
    drakeSystemAnalysis
    drakeSystemControllers
    robotlocomotion-lcmtypes-cpp)

  add_executable(kuka_simulation kuka_simulation.cc)
  target_link_libraries(kuka_simulation
    drakeSimDiagramBuilder
    drakeKukaIiwaArmCommon
    drakeLcmSystem
    drakeMultibodyParsers
    drakeRigidBodyPlant
    drakeSystemAnalysis
    drakeSystemControllers)
  drake_add_test(
      NAME kuka_simulation COMMAND kuka_simulation --simulation_sec 0.1)

  add_executable(iiwa_wsg_simulation iiwa_wsg_simulation.cc)
  target_link_libraries(iiwa_wsg_simulation
    drakeSimDiagramBuilder
    drakeKukaIiwaArmCommon
    drakeKukaIiwaWorld
    drakeLcmSystem
    drakeLCMUtil
    drakeRigidBodyPlant
    drakeSchunkWsg
    drakeSystemAnalysis
    drakeSystemControllers
    robotlocomotion-lcmtypes-cpp)
  drake_add_test(
      NAME iiwa_wsg_simulation COMMAND iiwa_wsg_simulation --simulation_sec 0.01)

  add_library_with_exports(LIB_NAME oracularStateEstimator SOURCE_FILES
      oracular_state_estimator.cc)
  target_link_libraries(oracularStateEstimator
      drakeLcmSystem
      drakeLCMUtil
      drakeRBM
      Eigen3::Eigen)
  drake_install_libraries(oracularStateEstimator)
  drake_install_headers(
      oracular_state_estimator.h)
  drake_install_pkg_config_file(oracular-state-estimator
      TARGET oracularStateEstimator
      LIBS -loracularStateEstimator
      REQUIRES drake-lcm-system drake-lcm-util drake-lcmtypes-cpp drake-rbm eigen3)

endif()

add_subdirectory(controlled_kuka)
add_subdirectory(iiwa_world)
add_subdirectory(dev)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
