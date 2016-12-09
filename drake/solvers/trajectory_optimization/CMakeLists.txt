add_library_with_exports(LIB_NAME drakeTrajectoryOptimization SOURCE_FILES
  dircol_trajectory_optimization.cc
  direct_trajectory_optimization.cc)
target_link_libraries(drakeTrajectoryOptimization
  drakeOptimization
  drakeTrajectories)
drake_install_libraries(drakeTrajectoryOptimization)
drake_install_headers(
  dircol_trajectory_optimization.h
  direct_trajectory_optimization.h)
drake_install_pkg_config_file(drake-trajectory-optimization
  TARGET drakeTrajectoryOptimization
  LIBS -ldrakeTrajectoryOptimization
  REQUIRES
    drake-optimization
    drake-trajectories
    eigen3)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
