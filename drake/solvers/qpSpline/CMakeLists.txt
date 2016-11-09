add_library_with_exports(LIB_NAME drakeSplineGeneration SOURCE_FILES splineGeneration.cpp SplineInformation.cpp ValueConstraint.cpp ContinuityConstraint.cpp)
target_link_libraries(drakeSplineGeneration drakeTrajectories)
pods_install_libraries(drakeSplineGeneration)
drake_install_headers(splineGeneration.h ValueConstraint.h ContinuityConstraint.h SplineInformation.h)
pods_install_pkg_config_file(drake-spline-generation
  LIBS -ldrakeSplineGeneration
  REQUIRES
  VERSION 0.0.2)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
