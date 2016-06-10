# Currently has alignment problems on 32 bit Windows apparently
# due to a bug in Eigen; see issue #2106 and PR #2107.
if(LCM_FOUND AND NOT (WIN32 AND (CMAKE_SIZEOF_VOID_P EQUAL 4)))
  add_executable(runQuadrotorDynamics runDynamics.cpp)
  target_link_libraries(runQuadrotorDynamics drakeRBSystem drakeLCMSystem)
  add_dependencies(runQuadrotorDynamics drake_lcmtypes lcmtype_agg_hpp)
  pods_use_pkg_config_packages(runQuadrotorDynamics lcm)
  add_test(NAME runQuadrotorDynamics COMMAND runQuadrotorDynamics 5.0 1.0 --non-realtime)

  add_executable(runQuadrotorLQR runLQR.cpp)
  target_link_libraries(runQuadrotorLQR drakeRBM)
  add_dependencies(runQuadrotorLQR drake_lcmtypes lcmtype_agg_hpp)
  pods_use_pkg_config_packages(runQuadrotorLQR lcm)
  add_test(NAME runQuadrotorLQR COMMAND runQuadrotorLQR --non-realtime)
endif()

# add_matlab_test(NAME examples/Quadrotor/Quadrotor.runOpenLoop COMMAND Quadrotor.runOpenLoop)  # FIXME
add_matlab_test(NAME examples/Quadrotor/runDircol COMMAND runDircol)
add_matlab_test(NAME examples/Quadrotor/runDircolWObs COMMAND runDircolWObs)
add_matlab_test(NAME examples/Quadrotor/runLQR COMMAND runLQR)
add_matlab_test(NAME examples/Quadrotor/runMixedIntegerSimpleForest COMMAND runMixedIntegerSimpleForest)

add_subdirectory(test)
add_subdirectory(crazyflie)
