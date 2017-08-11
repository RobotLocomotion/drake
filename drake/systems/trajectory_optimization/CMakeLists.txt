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

add_library_with_exports(LIB_NAME drakeContactImplicitConstraints SOURCE_FILES
        contact_implicit_constraints.cc)
target_link_libraries(drakeContactImplicitConstraints
        drakeOptimization
        drakeRBM)
drake_install_libraries(drakeContactImplicitConstraints)
drake_install_headers(
        contact_implicit_constraints.h
)
drake_install_pkg_config_file(drake-contact-implicit-constraints
        TARGET drakeContactImplicitConstraints
        LIBS -ldrakeContactImplicitConstraints
        REQUIRES
          drake-optimization
          drake-rbm)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
