add_library_with_exports(LIB_NAME drakeTrajectoryOptimization SOURCE_FILES
  direct_collocation.cc
  direct_collocation_constraint.cc
  direct_trajectory_optimization.cc)
target_link_libraries(drakeTrajectoryOptimization
  drakeOptimization
  drakeSystemFramework
  drakeTrajectories
  Eigen3::Eigen)
drake_install_libraries(drakeTrajectoryOptimization)
drake_install_headers(
  direct_collocation.h
  direct_collocation_constraint.h
  direct_trajectory_optimization.h)
drake_install_pkg_config_file(drake-trajectory-optimization
  TARGET drakeTrajectoryOptimization
  LIBS -ldrakeTrajectoryOptimization
  REQUIRES
    drake-optimization
    drake-system-framework
    drake-trajectories
    eigen3)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
