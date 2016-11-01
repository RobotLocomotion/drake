add_library_with_exports(LIB_NAME drakeTrajectories SOURCE_FILES
  PiecewiseFunction.cpp PiecewisePolynomialBase.cpp
  PiecewisePolynomial.cpp ExponentialPlusPiecewisePolynomial.cpp)
target_link_libraries(drakeTrajectories drakeCommon)
pods_install_libraries(drakeTrajectories)
drake_install_headers(PiecewiseFunction.h PiecewisePolynomial.h trajectory.h
  piecewise_polynomial_trajectory.h
  PiecewisePolynomialBase.h ExponentialPlusPiecewisePolynomial.h)
pods_install_pkg_config_file(drake-trajectories
  LIBS -ldrakeTrajectories
  REQUIRES
  VERSION 0.0.3)
add_subdirectory(test)
