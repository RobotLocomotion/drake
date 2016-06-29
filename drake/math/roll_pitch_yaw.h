#pragma once

#include <cmath>

#include <Eigen/Dense>

#include "drake/common/eigen_types.h"

namespace drake {
namespace math {

template <typename Derived>
drake::Vector4<typename Derived::Scalar> rpy2quat(
    const Eigen::MatrixBase<Derived>& rpy) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3);
  auto rpy_2 = (rpy / 2.0).array();
  auto s = rpy_2.sin();
  auto c = rpy_2.cos();

  Eigen::Vector4d q;
  q << c(0) * c(1) * c(2) + s(0) * s(1) * s(2),
      s(0) * c(1) * c(2) - c(0) * s(1) * s(2),
      c(0) * s(1) * c(2) + s(0) * c(1) * s(2),
      c(0) * c(1) * s(2) - s(0) * s(1) * c(2);

  q /= q.norm() + std::numeric_limits<typename Derived::Scalar>::epsilon();
  return q;
}

template <typename Derived>
drake::Vector4<typename Derived::Scalar> rpy2axis(
    const Eigen::MatrixBase<Derived>& rpy) {
  return quat2axis(rpy2quat(rpy));
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> rpy2rotmat(
    const Eigen::MatrixBase<Derived>& rpy) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3);
  auto rpy_array = rpy.array();
  auto s = rpy_array.sin();
  auto c = rpy_array.cos();

  Eigen::Matrix<typename Derived::Scalar, 3, 3> R;
  R.row(0) << c(2) * c(1), c(2) * s(1) * s(0) - s(2) * c(0),
      c(2) * s(1) * c(0) + s(2) * s(0);
  R.row(1) << s(2) * c(1), s(2) * s(1) * s(0) + c(2) * c(0),
      s(2) * s(1) * c(0) - c(2) * s(0);
  R.row(2) << -s(1), c(1) * s(0), c(1) * c(0);

  return R;
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 9, 3> drpy2rotmat(
    const Eigen::MatrixBase<Derived>& rpy) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3);
  auto rpy_array = rpy.array();
  auto s = rpy_array.sin();
  auto c = rpy_array.cos();

  Eigen::Matrix<typename Derived::Scalar, 9, 3> dR;
  dR.row(0) << 0, c(2) * -s(1), c(1) * -s(2);
  dR.row(1) << 0, -s(1) * s(2), c(2) * c(1);
  dR.row(2) << 0, -c(1), 0;
  dR.row(3) << c(2) * s(1) * c(0) - s(2) * -s(0), c(2) * c(1) * s(0),
      -s(2) * s(1) * s(0) - c(2) * c(0);
  dR.row(4) << s(2) * s(1) * c(0) + c(2) * -s(0), s(2) * c(1) * s(0),
      c(2) * s(1) * s(0) - s(2) * c(0);
  dR.row(5) << c(1) * c(0), -s(1) * s(0), 0;
  dR.row(6) << c(2) * s(1) * -s(0) + s(2) * c(0), c(2) * c(1) * c(0),
      -s(2) * s(1) * c(0) + c(2) * s(0);
  dR.row(7) << s(2) * s(1) * -s(0) - c(2) * c(0), s(2) * c(1) * c(0),
      c(2) * s(1) * c(0) + s(2) * s(0);
  dR.row(8) << c(1) * -s(0), -s(1) * c(0), 0;

  return dR;
}

}  // namespace math
}  // namespace drake
