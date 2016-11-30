#pragma once

#include <cmath>

#include <Eigen/Dense>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace math {
/**
 * Computes one of the quaternion from a rotation matrix.
 * This implementation is adapted from simbody
 * https://github.com/simbody/simbody/blob/master/SimTKcommon/Mechanics/src/Rotation.cpp
 * Notice that there are two quaternions corresponding to the same rotation,
 * namely `q` and `-q` represent the same rotation.
 * @param M A 3 x 3 rotation matrix.
 * @return a 4 x 1 unit length vector, the quaternion corresponding to the
 * rotation matrix.
 */
template <typename Derived>
Vector4<typename Derived::Scalar> rotmat2quat(
    const Eigen::MatrixBase<Derived>& M) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3, 3);

  typedef typename Derived::Scalar Scalar;

  Vector4<Scalar> q;

  // Check if the trace is larger than any diagonal
  Scalar tr = M.trace();
  if (tr >= M(0, 0) && tr >= M(1, 1) && tr >= M(2, 2)) {
    q(0) = 1 + tr;
    q(1) = M(2, 1) - M(1, 2);
    q(2) = M(0, 2) - M(2, 0);
    q(3) = M(1, 0) - M(0, 1);
  } else if (M(0, 0) >= M(1, 1) && M(0, 0) >= M(2, 2)) {
    q(0) = M(2, 1) - M(1, 2);
    q(1) = Scalar(1) - (tr - 2 * M(0, 0));
    q(2) = M(0, 1) + M(1, 0);
    q(3) = M(0, 2) + M(2, 0);
  } else if (M(1, 1) >= M(2, 2)) {
    q(0) = M(0, 2) - M(2, 0);
    q(1) = M(0, 1) + M(1, 0);
    q(2) = Scalar(1) - (tr - 2 * M(1, 1));
    q(3) = M(1, 2) + M(2, 1);
  } else {
    q(0) = M(1, 0) - M(0, 1);
    q(1) = M(0, 2) + M(2, 0);
    q(2) = M(1, 2) + M(2, 1);
    q(3) = 1 - (tr - 2 * M(2, 2));
  }
  Scalar scale = q.norm();
  q /= scale;
  return q;
}

/**
 * Computes the angle axis representation from a rotation matrix.
 * @tparam Derived An Eigen derived type, e.g., an Eigen Vector3d.
 * @param R  the 3 x 3 rotation matrix.
 * @return angle-axis representation, 4 x 1 vector as [x, y, z, angle].
 * [x, y, z] is a unit vector and 0 <= angle <= PI.
 */
template <typename Derived>
Vector4<typename Derived::Scalar> rotmat2axis(
    const Eigen::MatrixBase<Derived>& R) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3, 3);
  Eigen::AngleAxis<typename Derived::Scalar> angle_axis(R);

  // Before October 2016, Eigen calculated  0 <= angle <= 2*PI.
  // After  October 2016, Eigen calculates  0 <= angle <= PI.
  // Ensure consistency between pre/post October 2016 Eigen versions.
  using Scalar = typename Derived::Scalar;
  Scalar& angle = angle_axis.angle();
  Vector3<Scalar>& axis = angle_axis.axis();
  if (angle >= M_PI) {
    angle = 2 * M_PI - angle;
    axis = -axis;
  }

  Eigen::Vector4d aa;
  aa.head<3>() = axis;
  aa(3) = angle_axis.angle();
  DRAKE_ASSERT(0 <= aa(3) && aa(3) <= M_PI);
  return aa;
}

/**
 * Computes SpaceXYZ Euler angles from rotation matrix.
 * @tparam Derived An Eigen derived type, e.g., an Eigen Vector3d.
 * @param R 3x3 rotation matrix.
 * @return 3x1 SpaceXYZ Euler angles (called roll-pitch-yaw by ROS).
 * Note: SpaceXYZ roll-pitch-yaw is equivalent to BodyZYX yaw-pitch-roll.
 * http://answers.ros.org/question/58863/incorrect-rollpitch-yaw-values-using-getrpy/
 * @see rpy2rotmat
 */
template <typename Derived>
Vector3<typename Derived::Scalar> rotmat2rpy(
    const Eigen::MatrixBase<Derived>& R) {
  // TO-DO(daihongkai@gmail.com) uncomment this block when the Eigen bug
  // http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1301
  // is fixed. Currently Eigen's EulerAngles does not guarantee the range of
  // the second angle covers PI.
  // Also delete Simbody's derived implementation

  /*EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3, 3);

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

  Scalar plusMinus = -1;
  Scalar minusPlus = 1;

  // Calculates theta2 using lots of information in the rotation matrix.
  Scalar Rsum = sqrt((R(i, i) * R(i, i) + R(i, j) * R(i, j) +
                      R(j, k) * R(j, k) + R(k, k) * R(k, k)) /
                     2);

  // Rsum = abs(cos(theta2)) is inherently positive.
  Scalar theta2 = atan2(plusMinus * R(i, k), Rsum);
  Scalar theta1, theta3;

  // There is a singularity when cos(theta2) == 0.
  if (Rsum > 4 * Eigen::NumTraits<Scalar>::epsilon()) {
    theta1 = atan2(minusPlus * R(j, k), R(k, k));
    theta3 = atan2(minusPlus * R(i, j), R(i, i));
  } else if (plusMinus * R(i, k) > 0) {
    // spos = 2*sin(theta1 + plusMinus*theta3)
    Scalar spos = R(j, i) + plusMinus * R(k, j);
    // cpos = 2*cos(theta1 + plusMinus*theta3)
    Scalar cpos = R(j, j) + minusPlus * R(k, i);
    Scalar theta1PlusMinusTheta3 = atan2(spos, cpos);
    theta1 = theta1PlusMinusTheta3;  // Arbitrary split
    theta3 = 0;                      // Arbitrary split
  } else {
    // sneg = 2*sin(theta1+minusPlus*theta3)
    Scalar sneg = plusMinus * (R(k, j) + minusPlus * R(j, i));
    // cneg = 2*cos(theta1+minusPlus*theta3)
    Scalar cneg = R(j, j) + plusMinus * R(k, i);
    Scalar theta1MinusPlusTheta3 = atan2(sneg, cneg);
    theta1 = theta1MinusPlusTheta3;  // Arbitrary split
    theta3 = 0;                      // Arbitrary split
  }

  // Return values have the following ranges
  // -pi   <= theta1 <= pi
  // -pi/2 <= theta2 <= pi/2
  // -pi   <= theta3 <= pi

  // Return in Drake/ROS conventional SpaceXYZ (roll-pitch-yaw) order
  // (which is equivalent to BodyZYX theta1, theta2, theta3 order).
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
