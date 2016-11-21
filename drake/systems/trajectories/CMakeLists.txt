add_library_with_exports(LIB_NAME drakeTrajectories SOURCE_FILES
  PiecewiseFunction.cpp PiecewisePolynomialBase.cpp
  PiecewisePolynomial.cpp ExponentialPlusPiecewisePolynomial.cpp
    piecewise_polynomial_trajectory.cc)
target_link_libraries(drakeTrajectories drakeCommon)
drake_install_libraries(drakeTrajectories)
drake_install_headers(PiecewiseFunction.h PiecewisePolynomial.h trajectory.h
  piecewise_polynomial_trajectory.h
  PiecewisePolynomialBase.h ExponentialPlusPiecewisePolynomial.h)
drake_install_pkg_config_file(drake-trajectories
  TARGET drakeTrajectories
  LIBS -ldrakeTrajectories
  REQUIRES
    drake-common
    eigen3)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
