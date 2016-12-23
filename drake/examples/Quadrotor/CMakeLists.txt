# Currently has alignment problems on 32 bit Windows apparently
# due to a bug in Eigen; see issue #2106 and PR #2107.
if(lcm_FOUND)
  add_executable(run_quadrotor_dynamics run_quadrotor_dynamics.cc)
  target_link_libraries(run_quadrotor_dynamics
      drakeLCMSystem2
      drakeMultibodyParsers
      drakeRigidBodyPlant
      drakeSystemAnalysis
      drakeSystemPrimitives
      gflags)
  drake_add_test(NAME run_quadrotor_dynamics COMMAND run_quadrotor_dynamics --duration=1.0 SIZE medium)
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
