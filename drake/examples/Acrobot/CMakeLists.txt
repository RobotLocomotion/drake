add_library_with_exports(LIB_NAME drakeAcrobotPlant
    SOURCE_FILES acrobot_plant.cc)
target_link_libraries(drakeAcrobotPlant drakeSystemFramework)

if(lcm_FOUND)
  add_executable(acrobot_run_dynamics acrobot_run_dynamics.cc)
  target_link_libraries(acrobot_run_dynamics
      drakeAcrobotPlant
      drakeRigidBodyPlant
      drakeSystemAnalysis)
  drake_add_test(NAME acrobot_run_dynamics
      COMMAND acrobot_run_dynamics)
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

