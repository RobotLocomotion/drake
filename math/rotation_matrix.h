#pragma once

#include <cmath>
#include <limits>

#include <Eigen/Dense>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
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

/// Computes the rotation matrix for rotating by theta (radians) around the
/// positive X axis.
template <typename T>
Matrix3<T> XRotation(const T& theta) {
  Matrix3<T> R;
  using std::sin;
  using std::cos;
  const T c = cos(theta), s = sin(theta);
  // clang-format off
  R << 1, 0,  0,
       0, c, -s,
       0, s,  c;
  // clang-format on
  return R;
}

/// Computes the rotation matrix for rotating by theta (radians) around the
/// positive Y axis.
template <typename T>
Matrix3<T> YRotation(const T& theta) {
  Matrix3<T> R;
  using std::sin;
  using std::cos;
  const T c = cos(theta), s = sin(theta);
  // clang-format off
  R <<  c, 0, s,
        0, 1, 0,
       -s, 0, c;
  // clang-format on
  return R;
}

/// Computes the rotation matrix for rotating by theta (radians) around the
/// positive Z axis.
template <typename T>
Matrix3<T> ZRotation(const T& theta) {
  Matrix3<T> R;
  using std::sin;
  using std::cos;
  const T c = cos(theta), s = sin(theta);
  // clang-format off
  R << c, -s, 0,
       s,  c, 0,
       0,  0,  1;
  // clang-format on
  return R;
}

/// Projects a full-rank 3x3 matrix @p M onto O(3), defined as
/// <pre>
///   min_R  \sum_i,j | R(i,j) - M(i,j) |^2
///  subject to   R*R^T = I  =>  R ∈ O(3)
/// </pre>
///
/// The algorithm (just SVD) can be derived as a small modification of
/// section 3.2 in http://haralick.org/conferences/pose_estimation.pdf .
///
/// Note that it does not enforce det(R)=1; you could get det(R)=-1 if that
/// solution is closer to the matrix M using the norm above.
template <typename Derived>
Matrix3<typename Derived::Scalar> ProjectMatToOrthonormalMat(
    const Eigen::MatrixBase<Derived>& M) {
  DRAKE_DEMAND(M.rows() == 3 && M.cols() == 3);
  const auto svd = M.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
  return svd.matrixU() * svd.matrixV().transpose();
}

/// Projects a full-rank 3x3 matrix @p M onto SO(3), defined as
/// <pre>
///   min_R  \sum_i,j | R(i,j) - M(i,j) |^2
///  subject to   R*R^T = I, det(R)=1  =>  R ∈ SO(3)
/// </pre>
///
/// This algorithm is from Section 3.1, Eq. (3.7), of:
///   Moakher M (2002). "Means and averaging in the group of rotations."
/// This reference was obtained from R's documentation and C++ implementation of
/// project_SO3C.
template <typename Derived>
Matrix3<typename Derived::Scalar> ProjectMatToRotMat(
    const Eigen::MatrixBase<Derived>& M) {
  DRAKE_DEMAND(M.rows() == 3 && M.cols() == 3);
  using Scalar = typename Derived::Scalar;
  const Scalar det = M.determinant();
  using std::abs;
  DRAKE_THROW_UNLESS(abs(det) > std::numeric_limits<Scalar>::epsilon());
  const Eigen::SelfAdjointEigenSolver<Matrix3<Scalar>> eig(M.transpose() * M);
  // Get reciprocal square root.
  Vector3<Scalar> L = eig.eigenvalues().array().rsqrt();
  if (det < 0) {
    // Flip the sign on the smallest eigenvalue. Note that
    // SelfAdjointEigenSolver sorts eigenvalues in real-value ascending order.
    L(0) *= -1;
  }
  const Matrix3<Scalar> U = eig.eigenvectors();
  return M * U * L.asDiagonal() * U.transpose();
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
