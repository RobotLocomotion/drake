add_library_with_exports(LIB_NAME drakeAcrobotPlant
    SOURCE_FILES acrobot_plant.cc)
target_link_libraries(drakeAcrobotPlant
    drakeLinearControl
    drakeMultibodyParsers
    drakeSensors
    drakeSystemFramework)

if(lcm_FOUND)
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

endif(lcm_FOUND)

drake_add_matlab_test(NAME examples/Acrobot/runLQR COMMAND runLQR)
drake_add_matlab_test(NAME examples/Acrobot/runMaxROA REQUIRES sedumi spotless OPTIONAL mosek COMMAND runMaxROA)
drake_add_matlab_test(NAME examples/Acrobot/runPassive COMMAND runPassive)
drake_add_matlab_test(NAME examples/Acrobot/runPassiveURDF OPTIONAL bullet COMMAND runPassiveURDF)
drake_add_matlab_test(NAME examples/Acrobot/runSwingUp REQUIRES snopt COMMAND runSwingUp)
drake_add_matlab_test(NAME examples/Acrobot/runTrigLQR REQUIRES spotless OPTIONAL bullet COMMAND runTrigLQR)
drake_add_matlab_test(NAME examples/Acrobot/runWProcessNoise OPTIONAL bullet COMMAND runWProcessNoise)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

