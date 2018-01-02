#pragma once

#include <Eigen/Dense>

#include "drake/common/constants.h"
#include "drake/math/gradient.h"
#include "drake/math/gradient_util.h"
#include "drake/math/normalize_vector.h"

namespace drake {
namespace math {
/**
 * Computes the gradient of the function that converts a unit length quaternion
 * to a rotation matrix.
 * @param quaternion A unit length quaternion [w;x;y;z]
 * @return The gradient
 */
template <typename Derived>
typename drake::math::Gradient<Eigen::Matrix<typename Derived::Scalar, 3, 3>,
                               drake::kQuaternionSize>::type
dquat2rotmat(const Eigen::MatrixBase<Derived>& quaternion) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>,
                                           drake::kQuaternionSize);

  typename drake::math::Gradient<Eigen::Matrix<typename Derived::Scalar, 3, 3>,
                                 drake::kQuaternionSize>::type ret;
  typename Eigen::MatrixBase<Derived>::PlainObject qtilde;
  typename drake::math::Gradient<Derived, drake::kQuaternionSize>::type dqtilde;
  drake::math::NormalizeVector(quaternion, qtilde, &dqtilde);

  typedef typename Derived::Scalar Scalar;
  Scalar w = qtilde(0);
  Scalar x = qtilde(1);
  Scalar y = qtilde(2);
  Scalar z = qtilde(3);

  ret << w, x, -y, -z, z, y, x, w, -y, z, -w, x, -z, y, x, -w, w, -x, y, -z, x,
      w, z, y, y, z, w, x, -x, -w, z, y, w, -x, -y, z;
  ret *= 2.0;
  ret *= dqtilde;
  return ret;
}

/**
 * Computes the gradient of the function that converts a rotation matrix to
 * body-fixed z-y'-x'' Euler angles.
 * @param R A 3 x 3 rotation matrix
 * @param dR A 9 x N matrix, dR(i,j) is the gradient of R(i) w.r.t x(j)
 * @return The gradient G. G is a 3 x N matrix.
 *   G(0,j) is the gradient of roll w.r.t x(j)
 *   G(1,j) is the gradient of pitch w.r.t x(j)
 *   G(2,j) is the gradient of yaw w.r.t x(j)
 */
template <typename DerivedR, typename DerivedDR>
typename drake::math::Gradient<
    Eigen::Matrix<typename DerivedR::Scalar, drake::kRpySize, 1>,
    DerivedDR::ColsAtCompileTime>::type
drotmat2rpy(const Eigen::MatrixBase<DerivedR>& R,
            const Eigen::MatrixBase<DerivedDR>& dR) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<DerivedR>,
                                           drake::kSpaceDimension,
                                           drake::kSpaceDimension);
  EIGEN_STATIC_ASSERT(
      Eigen::MatrixBase<DerivedDR>::RowsAtCompileTime == drake::kRotmatSize,
      THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);

  typename DerivedDR::Index nq = dR.cols();
  typedef typename DerivedR::Scalar Scalar;
  typedef
      typename drake::math::Gradient<Eigen::Matrix<Scalar, drake::kRpySize, 1>,
                                     DerivedDR::ColsAtCompileTime>::type
          ReturnType;
  ReturnType drpy(drake::kRpySize, nq);

  auto dR11_dq =
      getSubMatrixGradient<DerivedDR::ColsAtCompileTime>(dR, 0, 0, R.rows());
  auto dR21_dq =
      getSubMatrixGradient<DerivedDR::ColsAtCompileTime>(dR, 1, 0, R.rows());
  auto dR31_dq =
      getSubMatrixGradient<DerivedDR::ColsAtCompileTime>(dR, 2, 0, R.rows());
  auto dR32_dq =
      getSubMatrixGradient<DerivedDR::ColsAtCompileTime>(dR, 2, 1, R.rows());
  auto dR33_dq =
      getSubMatrixGradient<DerivedDR::ColsAtCompileTime>(dR, 2, 2, R.rows());

  Scalar sqterm = R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2);

  // droll_dq
  drpy.row(0) = (R(2, 2) * dR32_dq - R(2, 1) * dR33_dq) / sqterm;

  // dpitch_dq
  using namespace std;  // NOLINT(build/namespaces)
  Scalar sqrt_sqterm = sqrt(sqterm);
  drpy.row(1) =
      (-sqrt_sqterm * dR31_dq +
       R(2, 0) / sqrt_sqterm * (R(2, 1) * dR32_dq + R(2, 2) * dR33_dq)) /
      (R(2, 0) * R(2, 0) + R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2));

  // dyaw_dq
  sqterm = R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0);
  drpy.row(2) = (R(0, 0) * dR21_dq - R(1, 0) * dR11_dq) / sqterm;
  return drpy;
}

/**
 * Computes the gradient of the function that converts rotation matrix to
 * quaternion.
 * @param R A 3 x 3 rotation matrix
 * @param dR A 9 x N matrix, dR(i,j) is the gradient of R(i) w.r.t x_var(j)
 * @return The gradient G. G is a 4 x N matrix
 *   G(0,j) is the gradient of w w.r.t x_var(j)
 *   G(1,j) is the gradient of x w.r.t x_var(j)
 *   G(2,j) is the gradient of y w.r.t x_var(j)
 *   G(3,j) is the gradient of z w.r.t x_var(j)
 */
template <typename DerivedR, typename DerivedDR>
typename drake::math::Gradient<
    Eigen::Matrix<typename DerivedR::Scalar, drake::kQuaternionSize, 1>,
    DerivedDR::ColsAtCompileTime>::type
drotmat2quat(const Eigen::MatrixBase<DerivedR>& R,
             const Eigen::MatrixBase<DerivedDR>& dR) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<DerivedR>,
                                           drake::kSpaceDimension,
                                           drake::kSpaceDimension);
  EIGEN_STATIC_ASSERT(
      Eigen::MatrixBase<DerivedDR>::RowsAtCompileTime == drake::kRotmatSize,
      THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);

  typedef typename DerivedR::Scalar Scalar;
  typedef typename drake::math::Gradient<
      Eigen::Matrix<Scalar, drake::kQuaternionSize, 1>,
      DerivedDR::ColsAtCompileTime>::type ReturnType;
  typename DerivedDR::Index nq = dR.cols();

  auto dR11_dq =
      getSubMatrixGradient<DerivedDR::ColsAtCompileTime>(dR, 0, 0, R.rows());
  auto dR12_dq =
      getSubMatrixGradient<DerivedDR::ColsAtCompileTime>(dR, 0, 1, R.rows());
  auto dR13_dq =
      getSubMatrixGradient<DerivedDR::ColsAtCompileTime>(dR, 0, 2, R.rows());
  auto dR21_dq =
      getSubMatrixGradient<DerivedDR::ColsAtCompileTime>(dR, 1, 0, R.rows());
  auto dR22_dq =
      getSubMatrixGradient<DerivedDR::ColsAtCompileTime>(dR, 1, 1, R.rows());
  auto dR23_dq =
      getSubMatrixGradient<DerivedDR::ColsAtCompileTime>(dR, 1, 2, R.rows());
  auto dR31_dq =
      getSubMatrixGradient<DerivedDR::ColsAtCompileTime>(dR, 2, 0, R.rows());
  auto dR32_dq =
      getSubMatrixGradient<DerivedDR::ColsAtCompileTime>(dR, 2, 1, R.rows());
  auto dR33_dq =
      getSubMatrixGradient<DerivedDR::ColsAtCompileTime>(dR, 2, 2, R.rows());

  Eigen::Matrix<Scalar, 4, 3> A;
  A.row(0) << 1.0, 1.0, 1.0;
  A.row(1) << 1.0, -1.0, -1.0;
  A.row(2) << -1.0, 1.0, -1.0;
  A.row(3) << -1.0, -1.0, 1.0;
  Eigen::Matrix<Scalar, 4, 1> B = A * R.diagonal();
  typename Eigen::Matrix<Scalar, 4, 1>::Index ind, max_col;
  Scalar val = B.maxCoeff(&ind, &max_col);

  ReturnType dq(drake::kQuaternionSize, nq);
  using namespace std;  // NOLINT(build/namespaces)
  switch (ind) {
    case 0: {
      // val = trace(M)
      auto dvaldq = dR11_dq + dR22_dq + dR33_dq;
      auto dwdq = dvaldq / (4.0 * sqrt(1.0 + val));
      auto w = sqrt(1.0 + val) / 2.0;
      auto wsquare4 = 4.0 * w * w;
      dq.row(0) = dwdq;
      dq.row(1) =
          ((dR32_dq - dR23_dq) * w - (R(2, 1) - R(1, 2)) * dwdq) / wsquare4;
      dq.row(2) =
          ((dR13_dq - dR31_dq) * w - (R(0, 2) - R(2, 0)) * dwdq) / wsquare4;
      dq.row(3) =
          ((dR21_dq - dR12_dq) * w - (R(1, 0) - R(0, 1)) * dwdq) / wsquare4;
      break;
    }
    case 1: {
      // val = M(1, 1) - M(2, 2) - M(3, 3)
      auto dvaldq = dR11_dq - dR22_dq - dR33_dq;
      auto s = 2.0 * sqrt(1.0 + val);
      auto ssquare = s * s;
      auto dsdq = dvaldq / sqrt(1.0 + val);
      dq.row(0) =
          ((dR32_dq - dR23_dq) * s - (R(2, 1) - R(1, 2)) * dsdq) / ssquare;
      dq.row(1) = .25 * dsdq;
      dq.row(2) =
          ((dR12_dq + dR21_dq) * s - (R(0, 1) + R(1, 0)) * dsdq) / ssquare;
      dq.row(3) =
          ((dR13_dq + dR31_dq) * s - (R(0, 2) + R(2, 0)) * dsdq) / ssquare;
      break;
    }
    case 2: {
      // val = M(2, 2) - M(1, 1) - M(3, 3)
      auto dvaldq = -dR11_dq + dR22_dq - dR33_dq;
      auto s = 2.0 * (sqrt(1.0 + val));
      auto ssquare = s * s;
      auto dsdq = dvaldq / sqrt(1.0 + val);
      dq.row(0) =
          ((dR13_dq - dR31_dq) * s - (R(0, 2) - R(2, 0)) * dsdq) / ssquare;
      dq.row(1) =
          ((dR12_dq + dR21_dq) * s - (R(0, 1) + R(1, 0)) * dsdq) / ssquare;
      dq.row(2) = .25 * dsdq;
      dq.row(3) =
          ((dR23_dq + dR32_dq) * s - (R(1, 2) + R(2, 1)) * dsdq) / ssquare;
      break;
    }
    default: {
      // val = M(3, 3) - M(2, 2) - M(1, 1)
      auto dvaldq = -dR11_dq - dR22_dq + dR33_dq;
      auto s = 2.0 * (sqrt(1.0 + val));
      auto ssquare = s * s;
      auto dsdq = dvaldq / sqrt(1.0 + val);
      dq.row(0) =
          ((dR21_dq - dR12_dq) * s - (R(1, 0) - R(0, 1)) * dsdq) / ssquare;
      dq.row(1) =
          ((dR13_dq + dR31_dq) * s - (R(0, 2) + R(2, 0)) * dsdq) / ssquare;
      dq.row(2) =
          ((dR23_dq + dR32_dq) * s - (R(1, 2) + R(2, 1)) * dsdq) / ssquare;
      dq.row(3) = .25 * dsdq;
      break;
    }
  }
  return dq;
}
}  // namespace math
}  // namespace drake
