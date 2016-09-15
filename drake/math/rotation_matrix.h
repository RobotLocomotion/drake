#pragma once

#include <cmath>

#include <Eigen/Dense>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace math {

/** Adapts the code from
http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToAngle
This code handles the normal non-degenerate case, and the two degenerate
cases, when the rotation angle is either 0 or 180.
@param R  the 3 x 3 rotation matrix
@return the angle-axis representation, a 4 x 1 vector as [x;y;z;angle]. The
axis [x;y;z] has unit length
*/
template <typename Derived>
Vector4<typename Derived::Scalar> rotmat2axis(
    const Eigen::MatrixBase<Derived>& R) {
  using std::sqrt;
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3, 3);
  Eigen::Vector4d a;
  typename Derived::Scalar epsilon_scalar = std::numeric_limits<typename Derived::Scalar>::epsilon();
  if(std::abs(R(0, 1) - R(1, 0)) < epsilon_scalar
      && std::abs(R(0, 2) - R(2, 0)) < epsilon_scalar
      && std::abs(R(1, 2) - R(2, 1)) < epsilon_scalar) {
    // Singularity found
    // First checks for identity matrix
    if(CompareMatrices(R, drake::Matrix3<typename Derived::Scalar>::Identity(), std::numeric_limits<typename Derived::Scalar>::epsilon(), MatrixCompareType::absolute)) {
      a << 1.0, 0.0, 0.0, 0.0;
    }
    else {
      // singularity is angle = 180
      typename Derived::Scalar xx = (R(0, 0) + 1.0) / 2;
      typename Derived::Scalar yy = (R(1, 1) + 1.0) / 2;
      typename Derived::Scalar zz = (R(2, 2) + 1.0) / 2;
      typename Derived::Scalar xy = (R(0, 1) + R(1, 0)) / 4;
      typename Derived::Scalar xz = (R(0, 2) + R(2, 0)) / 4;
      typename Derived::Scalar yz = (R(1, 2) + R(2, 1)) / 4;
      typename Derived::Scalar x, y, z;
      double sqrt_2 = std::sqrt(2);
      if ((xx > yy) && (xx > zz)) {
        // R(0, 0) is the largest diagonal entry
        if (xx < epsilon_scalar) {
          x = 0.0;
          y = sqrt_2/2;
          z = sqrt_2/2;
        }
        else {
          x = std::sqrt(xx);
          y = xy / x;
          z = xz / x;
        }
      }
      else if (yy > zz) {
        // R(1, 1) is the largest diagonal entry
        if(yy < epsilon_scalar) {
          x = sqrt_2/2;
          y = 0.0;
          z = sqrt_2/2;
        }
        else {
          y = std::sqrt(yy);
          x = xy / y;
          z = yz / y;
        }
      }
      else {
        // R(2,2) is the largest diagonal entry
        if (zz < epsilon_scalar) {
          x = sqrt_2/2;
          y = sqrt_2/2;
          z = 0;
        }
        else {
          z = std::sqrt(zz);
          x = xz / z;
          y = yz / z;
        }
      }
      a << x, y, z, M_PI;
    }
  }
  else {
    typename Derived::Scalar theta = acos((R.trace() - 1.0) / 2.0);
    a << R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1), theta;
    a.head<3>() *= 1.0 / (2.0 * sin(theta));
  }
  return a;
}

template <typename Derived>
Vector4<typename Derived::Scalar> rotmat2quat(
    const Eigen::MatrixBase<Derived>& M) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3, 3);

  typedef typename Derived::Scalar Scalar;
  Eigen::Matrix<Scalar, 4, 3> A;
  A.row(0) << 1.0, 1.0, 1.0;
  A.row(1) << 1.0, -1.0, -1.0;
  A.row(2) << -1.0, 1.0, -1.0;
  A.row(3) << -1.0, -1.0, 1.0;
  Vector4<Scalar> B = A * M.diagonal();
  Eigen::Index ind, max_col;
  Scalar val = B.maxCoeff(&ind, &max_col);

  Scalar w, x, y, z;
  switch (ind) {
    case 0: {
      // val = trace(M)
      w = sqrt(1.0 + val) / 2.0;
      Scalar w4 = w * 4.0;
      x = (M(2, 1) - M(1, 2)) / w4;
      y = (M(0, 2) - M(2, 0)) / w4;
      z = (M(1, 0) - M(0, 1)) / w4;
      break;
    }
    case 1: {
      // val = M(1, 1) - M(2, 2) - M(3, 3)
      Scalar s = 2.0 * sqrt(1.0 + val);
      w = (M(2, 1) - M(1, 2)) / s;
      x = 0.25 * s;
      y = (M(0, 1) + M(1, 0)) / s;
      z = (M(0, 2) + M(2, 0)) / s;
      break;
    }
    case 2: {
      //  % val = M(2, 2) - M(1, 1) - M(3, 3)
      Scalar s = 2.0 * (sqrt(1.0 + val));
      w = (M(0, 2) - M(2, 0)) / s;
      x = (M(0, 1) + M(1, 0)) / s;
      y = 0.25 * s;
      z = (M(1, 2) + M(2, 1)) / s;
      break;
    }
    default: {
      // val = M(3, 3) - M(2, 2) - M(1, 1)
      Scalar s = 2.0 * (sqrt(1.0 + val));
      w = (M(1, 0) - M(0, 1)) / s;
      x = (M(0, 2) + M(2, 0)) / s;
      y = (M(1, 2) + M(2, 1)) / s;
      z = 0.25 * s;
      break;
    }
  }

  Vector4<Scalar> q;
  q << w, x, y, z;
  return q;
}

template <typename Derived>
Vector3<typename Derived::Scalar> rotmat2rpy(
    const Eigen::MatrixBase<Derived>& R) {
  using std::atan2;
  using std::pow;
  using std::sqrt;
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3, 3);

  Vector3<typename Derived::Scalar> rpy;
  rpy << atan2(R(2, 1), R(2, 2)),
      atan2(-R(2, 0), sqrt(pow(R(2, 1), 2.0) + pow(R(2, 2), 2.0))),
      atan2(R(1, 0), R(0, 0));
  return rpy;
}

template <typename Derived>
VectorX<typename Derived::Scalar> rotmat2Representation(
    const Eigen::MatrixBase<Derived>& R, int rotation_type) {
  typedef typename Derived::Scalar Scalar;
  switch (rotation_type) {
    case 0:
      return Eigen::Matrix<Scalar, Eigen::Dynamic, 1>(0, 1);
    case 1:
      return rotmat2rpy(R);
    case 2:
      return rotmat2quat(R);
    default:
      throw std::runtime_error("rotation representation type not recognized");
  }
}

}  // namespace math
}  // namespace drake
