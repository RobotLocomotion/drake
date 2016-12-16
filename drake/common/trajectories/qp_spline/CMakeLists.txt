add_library_with_exports(LIB_NAME drakeSplineGeneration SOURCE_FILES
    continuity_constraint.cc
    spline_generation.cc
    spline_information.cc
    value_constraint.cc
    )
target_link_libraries(drakeSplineGeneration
  drakeTrajectories
  Eigen3::Eigen)
drake_install_libraries(drakeSplineGeneration)
drake_install_headers(
    continuity_constraint.h
    spline_generation.h
    spline_information.h
    value_constraint.h
    )
drake_install_pkg_config_file(drake-spline-generation
  TARGET drakeSplineGeneration
  LIBS -ldrakeSplineGeneration
  REQUIRES
    drake-trajectories
    eigen3)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
