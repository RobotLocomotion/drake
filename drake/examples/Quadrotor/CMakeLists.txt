# Currently has alignment problems on 32 bit Windows apparently
# due to a bug in Eigen; see issue #2106 and PR #2107.

if(lcm_FOUND)
  add_library_with_exports(LIB_NAME drakeQuadrotorCommon SOURCE_FILES
      quadrotor_plant.cc)
  target_link_libraries(drakeQuadrotorCommon
      drakeLinearControl
      drakeLCMSystem2
      drakeRBM)
  drake_install_libraries(drakeQuadrotorCommon)
  drake_install_headers(
      quadrotor_plant.h)
  drake_install_pkg_config_file(drake-quadrotor-common
      TARGET drakeQuadrotorCommon
      LIBS -ldrakeQuadrotorCommon
      REQUIRES drake-lcm-system
      drake-linear-controller
      drake-lcmtypes-cpp
      drake-rbm)

  add_executable(run_quadrotor_dynamics run_quadrotor_dynamics.cc)
  target_link_libraries(run_quadrotor_dynamics
      drakeLCMSystem2
      drakeMultibodyParsers
      drakeRigidBodyPlant
      drakeSystemAnalysis
      drakeSystemPrimitives
      gflags)
  drake_add_test(NAME run_quadrotor_dynamics COMMAND run_quadrotor_dynamics --duration=1.0 SIZE medium)

  add_executable(run_quadrotor_lqr run_quadrotor_lqr.cc)
  target_link_libraries(run_quadrotor_lqr
      drakeQuadrotorCommon
      drakeLCMSystem2
      drakeMultibodyParsers
      drakeRigidBodyPlant
      drakeSystemAnalysis
      drakeSystemPrimitives
      gflags)
  drake_add_test(NAME run_quadrotor_lqr
      COMMAND run_quadrotor_lqr -simulation_trials 2 -simulation_real_time_rate 0.0)
endif()

# drake_add_matlab_test(NAME examples/Quadrotor/Quadrotor.runOpenLoop COMMAND Quadrotor.runOpenLoop)  # FIXME: see #3313
drake_add_matlab_test(NAME examples/Quadrotor/runDircol OPTIONAL bullet snopt COMMAND runDircol)
drake_add_matlab_test(NAME examples/Quadrotor/runDircolWObs REQUIRES bullet lcm libbot snopt COMMAND runDircolWObs SIZE large)
drake_add_matlab_test(NAME examples/Quadrotor/runLQR OPTIONAL bullet COMMAND runLQR)
drake_add_matlab_test(NAME examples/Quadrotor/runMixedIntegerSimpleForest REQUIRES iris lcm libbot mosek yalmip OPTIONAL bullet snopt COMMAND runMixedIntegerSimpleForest SIZE large)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

add_subdirectory(crazyflie)
