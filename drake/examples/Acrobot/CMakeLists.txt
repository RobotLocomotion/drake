add_library_with_exports(LIB_NAME drakeAcrobotPlant
        SOURCE_FILES acrobot_plant.cc acrobot_swing_up.cc)
target_link_libraries(drakeAcrobotPlant
        drakeLinearControl
        drakeMultibodyParsers
        drakeSensors
        drakeSystemFramework
        drakeTrajectoryOptimization
        Eigen3::Eigen)
drake_install_libraries(drakeAcrobotPlant)
drake_install_headers(acrobot_plant.h acrobot_swing_up.h)
drake_install_pkg_config_file(drake-acrobot-plant
        TARGET drakeAcrobotPlant
        LIBS -ldrakeAcrobotPlant
        REQUIRES
        drake-trajectory-optimization
        eigen3
        drake-linear-control
        drake-multibody-parsers
        drake-sensors
        drake-system-framework)

if(lcm_FOUND)
  add_library_with_exports(LIB_NAME drakeAcrobotLcm SOURCE_FILES
          acrobot_lcm.cc)
  target_link_libraries(drakeAcrobotLcm
          drakeLcmSystem)
  drake_install_libraries(drakeAcrobotLcm)
  drake_install_headers(
          acrobot_lcm.h)
  drake_install_pkg_config_file(drake-acrobot-lcm
          TARGET drakeAcrobotLcm
          LIBS -drakeAcrobotLcm
          REQUIRES drake-lcm-system drake-lcmtypes-cpp)

  add_executable(acrobot_lcm_msg_generator acrobot_lcm_msg_generator.cpp)
  target_link_libraries(acrobot_lcm_msg_generator
            drakeLCMTypes
            drakeLcm)

  add_executable(acrobot_plant_w_lcm acrobot_plant_w_lcm.cc)
  target_link_libraries(acrobot_plant_w_lcm
          drakeSystemAnalysis
          drakeLcmSystem
          drakeAcrobotLcm
          drakeAcrobotPlant
          drakeMultibodyParsers
          drakeRigidBodyPlant
          drakeSystemAnalysis
          drakeLinearControl)

  add_executable(acrobot_spong_controller_w_lcm
          acrobot_spong_controller_w_lcm.cc)
  target_link_libraries(acrobot_spong_controller_w_lcm
          drakeSystemAnalysis
          drakeLcmSystem
          drakeAcrobotLcm
          drakeAcrobotPlant)

  add_executable(acrobot_run_passive acrobot_run_passive.cc)
  target_link_libraries(acrobot_run_passive
      drakeAcrobotPlant
      drakeRigidBodyPlant
      drakeSystemAnalysis)
  drake_add_test(NAME acrobot_run_passive COMMAND acrobot_run_passive -realtime_factor 0.0)

  add_executable(acrobot_run_lqr acrobot_run_lqr.cc)
  target_link_libraries(acrobot_run_lqr
      drakeAcrobotPlant
      drakeMultibodyParsers
      drakeRigidBodyPlant
      drakeSystemAnalysis)
  drake_add_test(NAME acrobot_run_lqr COMMAND acrobot_run_lqr -realtime_factor 0.0)

  add_executable(acrobot_run_lqr_w_estimator acrobot_run_lqr_w_estimator.cc)
  target_link_libraries(acrobot_run_lqr_w_estimator
      drakeAcrobotPlant
      drakeEstimators
      drakeMultibodyParsers
      drakeRigidBodyPlant
      drakeSensors
      drakeSystemAnalysis)
  drake_add_test(NAME acrobot_run_lqr_w_estimator COMMAND acrobot_run_lqr_w_estimator -realtime_factor 0.0)

  add_executable(acrobot_run_swing_up acrobot_run_swing_up.cc)
  target_link_libraries(acrobot_run_swing_up
          drakeAcrobotPlant
          drakeMultibodyParsers
          drakeRigidBodyPlant
          drakeSystemAnalysis
          drakeLinearControl)

  add_executable(acrobot_run_swing_up_traj_optimization
          acrobot_run_swing_up_traj_optimization.cc)
  target_link_libraries(acrobot_run_swing_up_traj_optimization
          drakeAcrobotPlant
          drakeRigidBodyPlant
          drakeSystemAnalysis)

endif(lcm_FOUND)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

