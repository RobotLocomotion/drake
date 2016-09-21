#pragma once

#include <cmath>

#include <Eigen/Dense>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace math {
/** Computes one of the quaternion from a rotation matrix.
 * The implementation is adapted from
 * http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
 * Notice that there are two quaternions corresponding to the same rotation,
 * namely @p q and @p -q represent the same rotation.
 * @param M A 3 x 3 rotation matrix.
 * @return a 4 x 1 unit length vector, the quaternion corresponding to the
 * rotation matrix
 */
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

/** Computes the angle axis representation from a rotation matrix. Since our
 * ::rotmat2quat and ::quat2axis are both numerical stable (they handle all the
 * corner cases, and avoid calling acos), we will call these two transform
 * functions directly.
 * @param R  the 3 x 3 rotation matrix
 * @return the angle-axis representation, a 4 x 1 vector as [x;y;z;angle]. The
 * axis [x;y;z] has unit length, the angle satisfies -PI < angle <= PI
 */
template <typename Derived>
Vector4<typename Derived::Scalar> rotmat2axis(
    const Eigen::MatrixBase<Derived>& R) {
  Eigen::AngleAxis<typename Derived::Scalar> a_eigen(R);
  Eigen::Vector4d a;
  a.head<3>() = a_eigen.axis();
  a(3) = a_eigen.angle();
  return a;
}

/**
 * Compute the Euler angles from rotation matrix
 * @param R A 3 x 3 rotation matrix
 * @return A 3 x 1 Euler angles about Body-fixed z-y'-x'' axes by [rpy(2), rpy(1), rpy(0)]
 * @see rpy2rotmat
 */
template <typename Derived>
Vector3<typename Derived::Scalar> rotmat2rpy(
    const Eigen::MatrixBase<Derived>& R) {
  // TO-DO(daihongkai@gmail.com) uncomment this block when the Eigen bug
  // http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1301
  // is fixed. Currently Eigen's EulerAngles does not guarantee the range of
  // the second angle covers PI.
  /*
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3, 3);

  auto euler_angles =
      Eigen::EulerAngles<typename Derived::Scalar, Eigen::EulerSystemZYX>::
          template FromRotation<false, false, false>(R);
  return drake::Vector3<typename Derived::Scalar>(
      euler_angles.gamma(), euler_angles.beta(), euler_angles.alpha());
  */

  // This implementation is adapted from simbody
  // https://github.com/simbody/simbody/blob/master/SimTKcommon/Mechanics/src/Rotation.cpp
  using std::atan2;
  using std::sqrt;
  using Scalar = typename Derived::Scalar;
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3, 3);

  int i = 2;
  int j = 1;
  int k = 0;


  // Calculate theta2 using lots of information in the rotation matrix
  Scalar Rsum = sqrt((R(i,i)*R(i,i) + R(i,j)*R(i,j) + R(j,k)*R(j,k) + R(k,k) * R(k,k))/2);

  // Rsum = abs(cos(theta2)) is inherently positive
  Scalar theta2 = atan2(R(i,k), Rsum);
  Scalar theta1, theta3;

  // There is a singularity when cos(theta2) == 0
  if(Rsum > 4 * std::numeric_limits<Scalar>::epsilon()) {
    theta1 = atan2(-R(j,k), R(k,k));
    theta3 = atan2(-R(i,j), R(i,i));
  }
  else if(R(i,k) > 0) {
    // spos = 2*sin(theta1 + plusMinus*theta3)
    Scalar spos = R(j,i) + R(k,j);
    // cpos = 2*cos(theta1 + plusMinus*theta3)
    Scalar cpos = R(j,j) -R(k,i);
    Scalar theta1PlusMinusTheta3 = atan2(spos, cpos);
    theta1 = theta1PlusMinusTheta3; // Arbitrary split
    theta3 = 0;                     // Arbitrary split
  }
  else {
    // sneg = 2*sin(theta1+minusPlus*theta3)
    Scalar sneg = R(k,j) - R(j,i);
    // cneg = 2*cos(theta1+minusPlus*theta3)
    Scalar cneg = R(j,j) + R(k,i);
    Scalar theta1MinusPlusTheta3 = atan2(sneg, cneg);
    theta1 = theta1MinusPlusTheta3; // Arbitrary split
    theta3 = 0;                     // Arbitrary split
  }

  // Return values have the following ranges
  // -pi   <  theta1 <= pi
  // -pi/2 <= theta2 <= pi/2
  // -pi   <  theta3 <= pi
  return drake::Vector3<Scalar>(theta3, theta2, theta1);
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
