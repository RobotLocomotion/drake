add_library_with_exports(LIB_NAME drakeSplineGeneration SOURCE_FILES splineGeneration.cpp SplineInformation.cpp ValueConstraint.cpp ContinuityConstraint.cpp)
target_link_libraries(drakeSplineGeneration drakeTrajectories)
drake_install_libraries(drakeSplineGeneration)
drake_install_headers(splineGeneration.h ValueConstraint.h ContinuityConstraint.h SplineInformation.h)
drake_install_pkg_config_file(drake-spline-generation
  TARGET drakeSplineGeneration
  LIBS -ldrakeSplineGeneration
  REQUIRES
    drake-trajectories
    eigen3)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
