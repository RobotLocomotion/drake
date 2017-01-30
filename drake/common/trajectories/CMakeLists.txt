add_library_with_exports(LIB_NAME drakeTrajectories SOURCE_FILES
    exponential_plus_piecewise_polynomial.cc
    piecewise_function.cc
    piecewise_polynomial.cc
    piecewise_polynomial_base.cc
    piecewise_polynomial_trajectory.cc
    piecewise_quaternion.cc
    )
target_link_libraries(drakeTrajectories
  drakeCommon
  Eigen3::Eigen)
drake_install_libraries(drakeTrajectories)
drake_install_headers(
    exponential_plus_piecewise_polynomial.h
    piecewise_function.h
    piecewise_polynomial.h
    piecewise_polynomial_base.h
    piecewise_polynomial_trajectory.h
    trajectory.h
    piecewise_quaternion.h
    )
drake_install_pkg_config_file(drake-trajectories
  TARGET drakeTrajectories
  LIBS -ldrakeTrajectories
  REQUIRES
    drake-common
    eigen3)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

add_subdirectory(qp_spline)
