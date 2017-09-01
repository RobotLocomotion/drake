add_library_with_exports(LIB_NAME drakeTrajectoryOptimization SOURCE_FILES
  direct_collocation.cc
    multiple_shooting.cc)
target_link_libraries(drakeTrajectoryOptimization
  drakeOptimization
  drakeSystemFramework
  drakeTrajectories
  Eigen3::Eigen)
drake_install_libraries(drakeTrajectoryOptimization)
drake_install_headers(
  direct_collocation.h
    multiple_shooting.h)
drake_install_pkg_config_file(drake-trajectory-optimization
  TARGET drakeTrajectoryOptimization
  LIBS -ldrakeTrajectoryOptimization
  REQUIRES
    drake-optimization
    drake-system-framework
    drake-trajectories
    eigen3)

