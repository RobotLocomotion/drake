# Source files used to build drakeMath.
set(sources
  autodiff.cc
  autodiff_gradient.cc
  axis_angle.cc
  continuous_algebraic_ricatti_equation.cc
  cross_product.cc
  discrete_algebraic_riccati_equation.cc
  eigen_sparse_triplet.cc
  expmap.cc
  gradient.cc
  jacobian.cc
  matrix_util.cc
  normalize_vector.cc
  quaternion.cc
  roll_pitch_yaw.cc
  rotation_conversion_gradient.cc
  rotation_matrix.cc)

# Headers that should be installed with Drake so that they
# are available elsewhere via #include.
set(installed_headers
  autodiff.h
  autodiff_gradient.h
  axis_angle.h
  continuous_algebraic_ricatti_equation.h
  cross_product.h
  discrete_algebraic_riccati_equation.h
  eigen_sparse_triplet.h
  expmap.h
  gradient.h
  gradient_util.h
  jacobian.h
  matrix_util.h
  normalize_vector.h
  quaternion.h
  random_rotation.h
  roll_pitch_yaw.h
  roll_pitch_yaw_using_quaternion.h
  roll_pitch_yaw_not_using_quaternion.h
  rotation_conversion_gradient.h
  rotation_matrix.h
  saturate.h)

# Headers that are needed by code here but should not
# be exposed anywhere else.
set(private_headers)

add_library_with_exports(LIB_NAME drakeMath
  SOURCE_FILES ${sources} ${installed_headers} ${private_headers})
target_link_libraries(drakeMath
  drakeCommon
  Eigen3::Eigen)

drake_install_headers(${installed_headers})

drake_install_libraries(drakeMath)
drake_install_pkg_config_file(drake-math
  TARGET drakeMath
  LIBS -ldrakeMath
  REQUIRES
    drake-common
    eigen3)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
