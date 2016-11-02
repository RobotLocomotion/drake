# Currently has alignment problems on 32 bit Windows apparently
# due to a bug in Eigen; see issue #2106 and PR #2107.
if(lcm_FOUND AND NOT (WIN32 AND (CMAKE_SIZEOF_VOID_P EQUAL 4)))
  add_executable(runQuadrotorDynamics runDynamics.cpp)
  target_link_libraries(runQuadrotorDynamics drakeRBSystem drakeLCMSystem drakeLCMTypes Threads::Threads)
  drake_add_test(NAME runQuadrotorDynamics COMMAND runQuadrotorDynamics 5.0 1.0 --non-realtime SIZE medium)

  add_executable(runQuadrotorLQR runLQR.cpp)
  target_link_libraries(runQuadrotorLQR drakeRBM drakeLCMTypes lcm)
  drake_add_test(NAME runQuadrotorLQR COMMAND runQuadrotorLQR --non-realtime SIZE medium)
endif()

# drake_add_matlab_test(NAME examples/Quadrotor/Quadrotor.runOpenLoop COMMAND Quadrotor.runOpenLoop)  # FIXME: see #3313
drake_add_matlab_test(NAME examples/Quadrotor/runDircol OPTIONAL bullet snopt COMMAND runDircol)
drake_add_matlab_test(NAME examples/Quadrotor/runDircolWObs REQUIRES bullet lcm libbot snopt COMMAND runDircolWObs SIZE large)
drake_add_matlab_test(NAME examples/Quadrotor/runLQR OPTIONAL bullet COMMAND runLQR)
drake_add_matlab_test(NAME examples/Quadrotor/runMixedIntegerSimpleForest REQUIRES iris lcm libbot mosek yalmip OPTIONAL bullet snopt COMMAND runMixedIntegerSimpleForest SIZE large)

add_subdirectory(test)
add_subdirectory(crazyflie)
