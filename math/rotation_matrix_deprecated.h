#pragma once

#ifndef DRAKE_MATH_ROTATION_MATRIX_DEPRECATED_HEADER_IS_ENABLED
// TODO(mitiguy): Delete this file when:
// * all the functions in this file have been updated/replaced by equivalent
//   functionality in rotation_matrix.h
// * all calls to the functions in the Drake code-base have been replaced by
//   calls to the corresponding functionality in rotation_matrix.h.
#error Include drake/math/rotation_matrix.h, not rotation_matrix_deprecated.h.
#endif

#include <cmath>
#include <limits>

#include <Eigen/Dense>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace math {
/// (Deprecated), use @ref math::RotationMatrix::ToQuaternion
// TODO(mitiguy) This code was deprecated on April 12, 2018.
// Delete this code in accordance with issue #8323.
template <typename Derived>
DRAKE_DEPRECATED("This code is deprecated per issue #8323. "
                     "Use RotationMatrix::ToQuaternion.")
Vector4<typename Derived::Scalar> rotmat2quat(
    const Eigen::MatrixBase<Derived>& M) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3, 3);

  using Scalar = typename Derived::Scalar;
  const drake::math::RotationMatrix<Scalar> R((drake::Matrix3<Scalar>(M)));
  return R.ToQuaternionAsVector4();
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

/// (Deprecated), use @ref math::RotationMatrix or @ref math::rotmat2rpy
/// or @ref math::RotationMatrix::ToQuaternionAsVector4.
// TODO(mitiguy) This code was deprecated on April 12, 2018.
// Even prior to this date, there are no calls in drake to this function.
// Delete this code in accordance with issue #8323.
template <typename Derived>
DRAKE_DEPRECATED("This code is deprecated per issue #8323. "
                 "Use math::RotationMatrix(R) or math::rotmat2rpy(R) "
                 "or math::RotationMatrix::ToQuaternionAsVector4.")
VectorX<typename Derived::Scalar> rotmat2Representation(
    const Eigen::MatrixBase<Derived>& R, int rotation_type) {
  throw std::runtime_error("This code is deprecated per issue #8323. "
                           "Use math::RotationMatrix(R) or math::rotmat2rpy(R) "
                           "or math::RotationMatrix::ToQuaternionAsVector4.");
}

/// (Deprecated), use @ref math::RotationMatrix::MakeXRotation
template <typename T>
Matrix3<T> XRotation(const T& theta) {
  return drake::math::RotationMatrix<T>::MakeXRotation(theta).matrix();
}

/// (Deprecated), use @ref math::RotationMatrix::MakeYRotation
template <typename T>
Matrix3<T> YRotation(const T& theta) {
  return drake::math::RotationMatrix<T>::MakeYRotation(theta).matrix();
}

/// (Deprecated), use @ref math::RotationMatrix::MakeZRotation
template <typename T>
Matrix3<T> ZRotation(const T& theta) {
  return drake::math::RotationMatrix<T>::MakeZRotation(theta).matrix();
}

/// (Deprecated), use @ref math::RotationMatrix::ProjectToRotationMatrix
// TODO(mitiguy) This code was deprecated on March 9, 2018.
// Delete this code in accordance with issue #8323.
template <typename Derived>
DRAKE_DEPRECATED("Use RotationMatrix::ProjectToRotationMatrix().")
Matrix3<typename Derived::Scalar> ProjectMatToOrthonormalMat(
    const Eigen::MatrixBase<Derived>& M) {
  using Scalar = typename Derived::Scalar;
  const drake::Matrix3<Scalar> R = M;
  return drake::math::RotationMatrix<Scalar>::ProjectToRotationMatrix(R,
      nullptr).matrix();
}

/// (Deprecated), use @ref math::RotationMatrix::ProjectToRotationMatrix
// TODO(mitiguy) This code was deprecated on March 9, 2018.
// Delete this code in accordance with issue #8323.
template <typename Derived>
DRAKE_DEPRECATED("Use RotationMatrix::ProjectToRotationMatrix().")
Matrix3<typename Derived::Scalar> ProjectMatToRotMat(
    const Eigen::MatrixBase<Derived>& M) {
  using Scalar = typename Derived::Scalar;
  const drake::Matrix3<Scalar> R = M;
  return drake::math::RotationMatrix<Scalar>::ProjectToRotationMatrix(R,
         nullptr).matrix();
}

/**
 * Projects a 3 x 3 matrix `M` onto SO(3). The projected rotation matrix `R`
 * has a given rotation axis `a`, and its rotation angle θ is bounded as
 * angle_lb <= θ <= angle_ub. One use case for this function is to reconstruct
 * the rotation matrix for a revolute joint with joint limits.
 * @see GlobalInverseKinematics for an usage of this function.
 * We can formulate this as an optimization problem
 * <pre>
 *   min_θ trace((R - M)ᵀ*(R - M))
 *   subject to R = I + sinθ * A + (1 - cosθ) * A²   (1)
 *              angle_lb <= θ <= angle_ub
 * </pre>
 * where `A` is the cross product matrix of the rotation axis `a`.
 * <pre>
 *   A = [ 0  -a₃  a₂]
 *       [ a₃  0  -a₁]
 *       [-a₂  a₁  0 ]
 * </pre>
 * Equation (1) is the Rodriguez Formula, to compute the rotation matrix from
 * the rotation axis `a` and the rotation angle θ. For more details, refer to
 * http://mathworld.wolfram.com/RodriguesRotationFormula.html
 * The objective function can be simplified as
 * <pre>
 *   max_θ trace(Rᵀ * M + Mᵀ * R)
 * </pre>
 * By substituting the matrix `R` with the axis-angle representation, the
 * optimization problem is formulated as
 * <pre>
 *   max_θ sinθ * trace(Aᵀ*M) - cosθ * trace(Mᵀ * A²)
 *   subject to angle_lb <= θ <= angle_ub
 * </pre>
 * By introducing α = atan2(-trace(Mᵀ * A²), trace(Aᵀ*M)), we can compute the
 * optimal θ as
 * <pre>
 * θ = π/2 + 2kπ - α, if angle_lb <= π/2 + 2kπ - α <= angle_ub, k ∈ ℤ
 * else
 * θ = angle_lb if sin(angle_lb + α) >= sin(angle_ub + α)
 * θ = angle_ub if sin(angle_lb + α) < sin(angle_ub + α)
 * </pre>
 * @tparam Derived A 3 x 3 matrix
 * @param M The matrix to be projected.
 * @param axis The axis of the rotation matrix. A unit length vector.
 * @param angle_lb The lower bound of the rotation angle.
 * @param angle_ub The upper bound of the rotation angle.
 * @return The rotation angle of the projected matrix.
 * @pre angle_ub >= angle_lb.
 * Throw std::runtime_error if these bounds are violated.
 */
template <typename Derived>
double ProjectMatToRotMatWithAxis(const Eigen::MatrixBase<Derived>& M,
                                  const Eigen::Ref<const Eigen::Vector3d>& axis,
                                  double angle_lb, double angle_ub) {
  using Scalar = typename Derived::Scalar;
  if (M.rows() != 3 || M.cols() != 3) {
    throw std::runtime_error("The input matrix should be of size 3 x 3.");
  }
  if (angle_ub < angle_lb) {
    throw std::runtime_error(
        "The angle upper bound should be no smaller than the angle lower "
        "bound.");
  }
  Vector3<Scalar> a = axis;
  a = a / a.norm();
  Eigen::Matrix3d A;
  // clang-format off
  A << 0, -a(2), a(1),
       a(2), 0, -a(0),
       -a(1), a(0), 0;
  // clang-format on
  Scalar alpha =
      atan2(-(M.transpose() * A * A).trace(), (A.transpose() * M).trace());
  Scalar theta{};
  // The bounds on θ + α is [angle_lb + α, angle_ub + α].
  if (std::isinf(angle_lb) && std::isinf(angle_ub)) {
    theta = M_PI_2 - alpha;
  } else if (std::isinf(angle_ub)) {
    // First if the angle upper bound is inf, start from the angle_lb, and
    // find the angle θ, such that θ + α = 0.5π + 2kπ
    int k = ceil((angle_lb + alpha - M_PI_2) / (2 * M_PI));
    theta = (2 * k + 0.5) * M_PI - alpha;
  } else if (std::isinf(angle_lb)) {
    // If the angle lower bound is inf, start from the angle_ub, and find the
    // angle θ, such that θ + α = 0.5π + 2kπ
    int k = floor((angle_ub + alpha - M_PI_2) / (2 * M_PI));
    theta = (2 * k + 0.5) * M_PI - alpha;
  } else {
    // Now neither angle_lb nor angle_ub is inf. Check if there exists an
    // integer k, such that 0.5π + 2kπ ∈ [angle_lb + α, angle_ub + α]
    int k = floor((angle_ub + alpha - M_PI_2) / (2 * M_PI));
    double max_sin_angle = M_PI_2 + 2 * k * M_PI;
    if (max_sin_angle >= angle_lb + alpha) {
      // 0.5π + 2kπ ∈ [angle_lb + α, angle_ub + α]
      theta = max_sin_angle - alpha;
    } else {
      // Now the maximal is at the boundary, either θ = angle_lb or angle_ub
      if (sin(angle_lb + alpha) >= sin(angle_ub + alpha)) {
        theta = angle_lb;
      } else {
        theta = angle_ub;
      }
    }
  }
  return theta;
}
}  // namespace math
}  // namespace drake
