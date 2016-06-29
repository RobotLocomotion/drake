/// @file
/// THIS FILE IS DEPRECATED.
/// Its contents are moving into drake/math.

#pragma once

#include <Eigen/Dense>
#include <cstring>
#include <cmath>
#include <random>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/drakeGeometryUtil_export.h"
#include "drake/math/gradient.h"
#include "drake/math/quaternion.h"
#include "drake/util/drakeGradientUtil.h"

const int QUAT_SIZE = 4;
const int EXPMAP_SIZE = 3;
const int HOMOGENEOUS_TRANSFORM_SIZE = 16;
const int AXIS_ANGLE_SIZE = 4;
const int SPACE_DIMENSION = 3;
const int RotmatSize = SPACE_DIMENSION * SPACE_DIMENSION;
const int RPY_SIZE = 3;

DRAKEGEOMETRYUTIL_EXPORT double angleDiff(double phi1, double phi2);

DRAKEGEOMETRYUTIL_EXPORT Eigen::Vector4d uniformlyRandomAxisAngle(
    std::default_random_engine& generator);
DRAKEGEOMETRYUTIL_EXPORT Eigen::Vector4d uniformlyRandomQuat(
    std::default_random_engine& generator);
DRAKEGEOMETRYUTIL_EXPORT Eigen::Matrix3d uniformlyRandomRotmat(
    std::default_random_engine& generator);
DRAKEGEOMETRYUTIL_EXPORT Eigen::Vector3d uniformlyRandomRPY(
    std::default_random_engine& generator);

// NOTE: not reshaping second derivative to Matlab geval output format!
template <typename Derived>
void normalizeVec(
    const Eigen::MatrixBase<Derived>& x, typename Derived::PlainObject& x_norm,
    typename drake::math::Gradient<Derived, Derived::RowsAtCompileTime,
                                   1>::type* dx_norm = nullptr,
    typename drake::math::Gradient<Derived, Derived::RowsAtCompileTime,
                                   2>::type* ddx_norm = nullptr) {
  typename Derived::Scalar xdotx = x.squaredNorm();
  typename Derived::Scalar norm_x = sqrt(xdotx);
  x_norm = x / norm_x;

  if (dx_norm) {
    dx_norm->setIdentity(x.rows(), x.rows());
    (*dx_norm) -= x * x.transpose() / xdotx;
    (*dx_norm) /= norm_x;

    if (ddx_norm) {
      auto dx_norm_transpose = transposeGrad(*dx_norm, x.rows());
      auto ddx_norm_times_norm = -matGradMultMat(x_norm, x_norm.transpose(),
                                                 (*dx_norm), dx_norm_transpose);
      auto dnorm_inv = -x.transpose() / (xdotx * norm_x);
      (*ddx_norm) = ddx_norm_times_norm / norm_x;
      auto temp = (*dx_norm) * norm_x;
      typename Derived::Index n = x.rows();
      for (int col = 0; col < n; col++) {
        auto column_as_matrix = (dnorm_inv(0, col) * temp);
        for (int row_block = 0; row_block < n; row_block++) {
          ddx_norm->block(row_block * n, col, n, 1) +=
              column_as_matrix.col(row_block);
        }
      }
    }
  }
}

DRAKEGEOMETRYUTIL_EXPORT int rotationRepresentationSize(int rotation_type);

/*
 * rotation conversion gradient functions
 */
template <typename Derived>
typename drake::math::Gradient<Eigen::Matrix<typename Derived::Scalar, 3, 3>,
                               QUAT_SIZE>::type
dquat2rotmat(const Eigen::MatrixBase<Derived>& q) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>,
                                           QUAT_SIZE);

  typename drake::math::Gradient<Eigen::Matrix<typename Derived::Scalar, 3, 3>,
                                 QUAT_SIZE>::type ret;
  typename Eigen::MatrixBase<Derived>::PlainObject qtilde;
  typename drake::math::Gradient<Derived, QUAT_SIZE>::type dqtilde;
  normalizeVec(q, qtilde, &dqtilde);

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

template <typename DerivedR, typename DerivedDR>
typename drake::math::Gradient<
    Eigen::Matrix<typename DerivedR::Scalar, RPY_SIZE, 1>,
    DerivedDR::ColsAtCompileTime>::type
drotmat2rpy(const Eigen::MatrixBase<DerivedR>& R,
            const Eigen::MatrixBase<DerivedDR>& dR) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<DerivedR>,
                                           SPACE_DIMENSION, SPACE_DIMENSION);
  EIGEN_STATIC_ASSERT(
      Eigen::MatrixBase<DerivedDR>::RowsAtCompileTime == RotmatSize,
      THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);

  typename DerivedDR::Index nq = dR.cols();
  typedef typename DerivedR::Scalar Scalar;
  typedef typename drake::math::Gradient<Eigen::Matrix<Scalar, RPY_SIZE, 1>,
                                         DerivedDR::ColsAtCompileTime>::type
      ReturnType;
  ReturnType drpy(RPY_SIZE, nq);

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

  using namespace std;
  // droll_dq
  drpy.row(0) = (R(2, 2) * dR32_dq - R(2, 1) * dR33_dq) / sqterm;

  // dpitch_dq
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

template <typename DerivedR, typename DerivedDR>
typename drake::math::Gradient<
    Eigen::Matrix<typename DerivedR::Scalar, QUAT_SIZE, 1>,
    DerivedDR::ColsAtCompileTime>::type
drotmat2quat(const Eigen::MatrixBase<DerivedR>& R,
             const Eigen::MatrixBase<DerivedDR>& dR) {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<DerivedR>,
                                           SPACE_DIMENSION, SPACE_DIMENSION);
  EIGEN_STATIC_ASSERT(
      Eigen::MatrixBase<DerivedDR>::RowsAtCompileTime == RotmatSize,
      THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);

  typedef typename DerivedR::Scalar Scalar;
  typedef typename drake::math::Gradient<Eigen::Matrix<Scalar, QUAT_SIZE, 1>,
                                         DerivedDR::ColsAtCompileTime>::type
      ReturnType;
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

  ReturnType dq(QUAT_SIZE, nq);
  using namespace std;
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

/*
 * cross product related
 */
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> vectorToSkewSymmetric(
    const Eigen::MatrixBase<Derived>& p) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>,
                                           SPACE_DIMENSION);
  Eigen::Matrix<typename Derived::Scalar, 3, 3> ret;
  ret << 0.0, -p(2), p(1), p(2), 0.0, -p(0), -p(1), p(0), 0.0;
  return ret;
}

template <typename DerivedA, typename DerivedB>
Eigen::Matrix<typename DerivedA::Scalar, 3, Eigen::Dynamic> dcrossProduct(
    const Eigen::MatrixBase<DerivedA>& a, const Eigen::MatrixBase<DerivedB>& b,
    const typename drake::math::Gradient<DerivedA, Eigen::Dynamic>::type& da,
    const typename drake::math::Gradient<DerivedB, Eigen::Dynamic>::type& db) {
  Eigen::Matrix<typename DerivedA::Scalar, 3, Eigen::Dynamic> ret(3, da.cols());
  ret.noalias() = da.colwise().cross(b);
  ret.noalias() -= db.colwise().cross(a);
  return ret;
}

/*
 * angular velocity conversion functions
 */
template <typename DerivedQ, typename DerivedM, typename DerivedDM>
void angularvel2quatdotMatrix(const Eigen::MatrixBase<DerivedQ>& q,
                              Eigen::MatrixBase<DerivedM>& M,
                              Eigen::MatrixBase<DerivedDM>* dM = nullptr) {
  // note: not normalizing to match MATLAB implementation
  using Scalar = typename DerivedQ::Scalar;
  M.resize(QUAT_SIZE, SPACE_DIMENSION);
  M.row(0) << -q(1), -q(2), -q(3);
  M.row(1) << q(0), q(3), -q(2);
  M.row(2) << -q(3), q(0), q(1);
  M.row(3) << q(2), -q(1), q(0);
  M *= Scalar(0.5);

  if (dM) {
    (*dM) << Scalar(0), Scalar(-0.5), Scalar(0), Scalar(0), Scalar(0.5),
        Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
        Scalar(-0.5), Scalar(0), Scalar(0), Scalar(0.5), Scalar(0), Scalar(0),
        Scalar(0), Scalar(-0.5), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
        Scalar(0.5), Scalar(0.5), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
        Scalar(-0.5), Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
        Scalar(-0.5), Scalar(0), Scalar(0), Scalar(-0.5), Scalar(0), Scalar(0),
        Scalar(0.5), Scalar(0), Scalar(0), Scalar(0.5), Scalar(0), Scalar(0),
        Scalar(0);
  }
}

template <typename DerivedRPY, typename DerivedPhi, typename DerivedDPhi,
          typename DerivedDDPhi>
void angularvel2rpydotMatrix(
    const Eigen::MatrixBase<DerivedRPY>& rpy,
    typename Eigen::MatrixBase<DerivedPhi>& phi,
    typename Eigen::MatrixBase<DerivedDPhi>* dphi = nullptr,
    typename Eigen::MatrixBase<DerivedDDPhi>* ddphi = nullptr) {
  phi.resize(RPY_SIZE, SPACE_DIMENSION);

  typedef typename DerivedRPY::Scalar Scalar;
  Scalar p = rpy(1);
  Scalar y = rpy(2);

  using namespace std;
  Scalar sy = sin(y);
  Scalar cy = cos(y);
  Scalar sp = sin(p);
  Scalar cp = cos(p);
  Scalar tp = sp / cp;

  phi << cy / cp, sy / cp, Scalar(0), -sy, cy, Scalar(0), cy * tp, tp * sy,
      Scalar(1);
  if (dphi) {
    dphi->resize(phi.size(), RPY_SIZE);
    Scalar sp2 = sp * sp;
    Scalar cp2 = cp * cp;
    (*dphi) << Scalar(0), (cy * sp) / cp2, -sy / cp, Scalar(0), Scalar(0), -cy,
        Scalar(0), cy + (cy * sp2) / cp2, -(sp * sy) / cp, Scalar(0),
        (sp * sy) / cp2, cy / cp, Scalar(0), Scalar(0), -sy, Scalar(0),
        sy + (sp2 * sy) / cp2, (cy * sp) / cp, Scalar(0), Scalar(0), Scalar(0),
        Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0);

    if (ddphi) {
      ddphi->resize(dphi->size(), RPY_SIZE);
      Scalar cp3 = cp2 * cp;
      (*ddphi) << Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
          Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
          Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
          Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
          Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
          -(cy * (cp2 - Scalar(2))) / cp3, (sp * sy) / (sp2 - Scalar(1)),
          Scalar(0), Scalar(0), Scalar(0), Scalar(0),
          (Scalar(2) * cy * sp) / cp3, sy / (sp2 - Scalar(1)), Scalar(0),
          (Scalar(2) * sy - cp2 * sy) / cp3, (cy * sp) / cp2, Scalar(0),
          Scalar(0), Scalar(0), Scalar(0), (Scalar(2) * sp * sy) / cp3,
          cy / cp2, Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
          Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
          (sp * sy) / (sp2 - Scalar(1)), -cy / cp, Scalar(0), Scalar(0), sy,
          Scalar(0), sy / (sp2 - Scalar(1)), -(cy * sp) / cp, Scalar(0),
          (cy * sp) / cp2, -sy / cp, Scalar(0), Scalar(0), -cy, Scalar(0),
          cy / cp2, -(sp * sy) / cp, Scalar(0), Scalar(0), Scalar(0), Scalar(0),
          Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0);
    }
  }
}

template <typename DerivedRPY, typename DerivedE>
void rpydot2angularvelMatrix(
    const Eigen::MatrixBase<DerivedRPY>& rpy, Eigen::MatrixBase<DerivedE>& E,
    typename drake::math::Gradient<DerivedE, RPY_SIZE, 1>::type* dE = nullptr) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<DerivedRPY>,
                                           RPY_SIZE);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<DerivedE>,
                                           SPACE_DIMENSION, RPY_SIZE);
  typedef typename DerivedRPY::Scalar Scalar;
  Scalar p = rpy(1);
  Scalar y = rpy(2);
  Scalar sp = sin(p);
  Scalar cp = cos(p);
  Scalar sy = sin(y);
  Scalar cy = cos(y);

  E << cp * cy, -sy, 0.0, cp * sy, cy, 0.0, -sp, 0.0, 1.0;
  if (dE) {
    (*dE) << 0.0, -sp * cy, -cp * sy, 0.0, -sp * sy, cp * cy, 0.0, -cp, 0.0,
        0.0, 0.0, -cy, 0.0, 0.0, -sy, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0;
  }
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 4> quatdot2angularvelMatrix(
    const Eigen::MatrixBase<Derived>& q) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>,
                                           QUAT_SIZE);
  typedef typename Derived::Scalar Scalar;
  auto qtilde = q.normalized();
  Eigen::Matrix<Scalar, 3, 4> ret;
  ret << -qtilde(1), qtilde(0), -qtilde(3), qtilde(2), -qtilde(2), qtilde(3),
      qtilde(0), -qtilde(1), -qtilde(3), -qtilde(2), qtilde(1), qtilde(0);
  ret *= Scalar(2);
  return ret;
}

template <typename DerivedRPY, typename DerivedRPYdot, typename DerivedOMEGA>
void rpydot2angularvel(
    const Eigen::MatrixBase<DerivedRPY>& rpy,
    const Eigen::MatrixBase<DerivedRPYdot>& rpydot,
    Eigen::MatrixBase<DerivedOMEGA>& omega,
    typename drake::math::Gradient<DerivedOMEGA, RPY_SIZE, 1>::type* domega =
        nullptr) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<DerivedRPY>,
                                           RPY_SIZE);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<DerivedRPYdot>,
                                           RPY_SIZE);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Eigen::MatrixBase<DerivedOMEGA>,
                                           RPY_SIZE, 1);

  Eigen::Matrix<typename DerivedOMEGA::Scalar, 3, 3> E;
  if (domega) {
    Eigen::Matrix<typename DerivedOMEGA::Scalar, 9, 3> dE;
    rpydot2angularvelMatrix(rpy, E, &dE);
    (*domega) << matGradMult(dE, rpydot), E;
  } else {
    rpydot2angularvelMatrix(rpy, E);
  }
  omega = E * rpydot;
}

/*
 * spatial transform functions
 */
template <typename Derived>
struct TransformSpatial {
  typedef typename Eigen::Matrix<typename Derived::Scalar, drake::kTwistSize,
                                 Derived::ColsAtCompileTime> type;
};

template <typename DerivedM>
typename TransformSpatial<DerivedM>::type transformSpatialMotion(
    const Eigen::Transform<typename DerivedM::Scalar, 3, Eigen::Isometry>& T,
    const Eigen::MatrixBase<DerivedM>& M) {
  Eigen::Matrix<typename DerivedM::Scalar, drake::kTwistSize,
                DerivedM::ColsAtCompileTime> ret(drake::kTwistSize, M.cols());
  ret.template topRows<3>().noalias() = T.linear() * M.template topRows<3>();
  ret.template bottomRows<3>().noalias() =
      -ret.template topRows<3>().colwise().cross(T.translation());
  ret.template bottomRows<3>().noalias() +=
      T.linear() * M.template bottomRows<3>();
  return ret;
}

template <typename Scalar, typename DerivedX, typename DerivedDT,
          typename DerivedDX>
typename drake::math::Gradient<DerivedX, DerivedDX::ColsAtCompileTime, 1>::type
dTransformSpatialMotion(const Eigen::Transform<Scalar, 3, Eigen::Isometry>& T,
                        const Eigen::MatrixBase<DerivedX>& X,
                        const Eigen::MatrixBase<DerivedDT>& dT,
                        const Eigen::MatrixBase<DerivedDX>& dX) {
  DRAKE_ASSERT(dT.cols() == dX.cols());
  typename DerivedDT::Index nq = dT.cols();

  const auto& R = T.linear();
  const auto& p = T.translation();

  std::array<int, 3> rows = {{0, 1, 2}};
  std::array<int, 3> R_cols = {{0, 1, 2}};
  std::array<int, 1> p_cols = {{3}};

  auto dR = getSubMatrixGradient<Eigen::Dynamic>(dT, rows, R_cols, T.Rows);
  auto dp = getSubMatrixGradient<Eigen::Dynamic>(dT, rows, p_cols, T.Rows);

  typename drake::math::Gradient<DerivedX, DerivedDX::ColsAtCompileTime,
                                 1>::type ret(X.size(), nq);
  std::array<int, 3> Xomega_rows = {{0, 1, 2}};
  std::array<int, 3> Xv_rows = {{3, 4, 5}};
  for (int col = 0; col < X.cols(); col++) {
    auto Xomega_col = X.template block<3, 1>(0, col);
    auto Xv_col = X.template block<3, 1>(3, col);

    auto RXomega_col = (R * Xomega_col).eval();

    std::array<int, 1> col_array = {{col}};
    auto dXomega_col = getSubMatrixGradient<Eigen::Dynamic>(
        dX, Xomega_rows, col_array, X.rows());
    auto dXv_col =
        getSubMatrixGradient<Eigen::Dynamic>(dX, Xv_rows, col_array, X.rows());

    auto domega_part_col =
        (R * dXomega_col + matGradMult(dR, Xomega_col)).eval();
    auto dv_part_col = (R * dXv_col + matGradMult(dR, Xv_col)).eval();
    dv_part_col += dp.colwise().cross(RXomega_col);
    dv_part_col -= domega_part_col.colwise().cross(p);

    setSubMatrixGradient<Eigen::Dynamic>(ret, domega_part_col, Xomega_rows,
                                         col_array, X.rows());
    setSubMatrixGradient<Eigen::Dynamic>(ret, dv_part_col, Xv_rows, col_array,
                                         X.rows());
  }
  return ret;
}

template <typename DerivedF>
typename TransformSpatial<DerivedF>::type transformSpatialForce(
    const Eigen::Transform<typename DerivedF::Scalar, 3, Eigen::Isometry>& T,
    const Eigen::MatrixBase<DerivedF>& F) {
  Eigen::Matrix<typename DerivedF::Scalar, drake::kTwistSize,
                DerivedF::ColsAtCompileTime> ret(drake::kTwistSize, F.cols());
  ret.template bottomRows<3>().noalias() =
      T.linear() * F.template bottomRows<3>().eval();
  ret.template topRows<3>() =
      -ret.template bottomRows<3>().colwise().cross(T.translation());
  ret.template topRows<3>().noalias() += T.linear() * F.template topRows<3>();
  return ret;
}

template <typename Scalar, typename DerivedX, typename DerivedDT,
          typename DerivedDX>
typename drake::math::Gradient<DerivedX, DerivedDX::ColsAtCompileTime>::type
dTransformSpatialForce(const Eigen::Transform<Scalar, 3, Eigen::Isometry>& T,
                       const Eigen::MatrixBase<DerivedX>& X,
                       const Eigen::MatrixBase<DerivedDT>& dT,
                       const Eigen::MatrixBase<DerivedDX>& dX) {
  DRAKE_ASSERT(dT.cols() == dX.cols());
  typename DerivedDT::Index nq = dT.cols();

  const auto& R = T.linear();
  const auto& p = T.translation();

  std::array<int, 3> rows = {{0, 1, 2}};
  std::array<int, 3> R_cols = {{0, 1, 2}};
  std::array<int, 1> p_cols = {{3}};

  auto dR = getSubMatrixGradient<Eigen::Dynamic>(dT, rows, R_cols, T.Rows);
  auto dp = getSubMatrixGradient<Eigen::Dynamic>(dT, rows, p_cols, T.Rows);

  typename drake::math::Gradient<DerivedX, DerivedDX::ColsAtCompileTime>::type
      ret(X.size(), nq);
  std::array<int, 3> Xomega_rows = {{0, 1, 2}};
  std::array<int, 3> Xv_rows = {{3, 4, 5}};
  for (int col = 0; col < X.cols(); col++) {
    auto Xomega_col = X.template block<3, 1>(0, col);
    auto Xv_col = X.template block<3, 1>(3, col);

    auto RXv_col = (R * Xv_col).eval();

    std::array<int, 1> col_array = {{col}};
    auto dXomega_col = getSubMatrixGradient<Eigen::Dynamic>(
        dX, Xomega_rows, col_array, X.rows());
    auto dXv_col =
        getSubMatrixGradient<Eigen::Dynamic>(dX, Xv_rows, col_array, X.rows());

    auto domega_part_col = (R * dXomega_col).eval();
    domega_part_col += matGradMult(dR, Xomega_col);
    auto dv_part_col = (R * dXv_col).eval();
    dv_part_col += matGradMult(dR, Xv_col);
    domega_part_col += dp.colwise().cross(RXv_col);
    domega_part_col -= dv_part_col.colwise().cross(p);

    setSubMatrixGradient<Eigen::Dynamic>(ret, domega_part_col, Xomega_rows,
                                         col_array, X.rows());
    setSubMatrixGradient<Eigen::Dynamic>(ret, dv_part_col, Xv_rows, col_array,
                                         X.rows());
  }
  return ret;
}

template <typename DerivedI>
bool isRegularInertiaMatrix(const Eigen::MatrixBase<DerivedI>& I) {
  using namespace Eigen;
  using Scalar = typename DerivedI::Scalar;
  bool ret = true;

  auto J = I.template topLeftCorner<3, 3>();
  auto cross_part_1 = I.template topRightCorner<3, 3>();
  auto cross_part_2 = I.template bottomLeftCorner<3, 3>();
  const auto& m = I(3, 3);
  ret = ret && (J - J.transpose()).isZero();  // J symmetric
  ret = ret &&
        (m * Matrix<Scalar, 3, 3>::Identity() -
         I.template bottomRightCorner<3, 3>())
            .isZero();  // mass part is a scalar matrix
  ret = ret &&
        (cross_part_1 - cross_part_2)
            .isZero();  // cross parts transposes of each other
  ret = ret &&
        (cross_part_1 + cross_part_1.transpose())
            .isZero();  // cross parts skew symmetric

  return ret;
}

template <typename DerivedI>
drake::SquareTwistMatrix<typename DerivedI::Scalar> transformSpatialInertia(
    const Eigen::Transform<typename DerivedI::Scalar, SPACE_DIMENSION,
                           Eigen::Isometry>& T_current_to_new,
    const Eigen::MatrixBase<DerivedI>& I) {
  using namespace Eigen;
  using Scalar = typename DerivedI::Scalar;

  if (isRegularInertiaMatrix(I)) {
    // this check is necessary to support the nonstandard inertia matrices
    // resulting from added masses

    // TODO(tkoolen): SpatialInertiaMatrix class that keeps track of whether
    // matrix is regular or not
    const auto& R = T_current_to_new.linear();
    const auto& p = T_current_to_new.translation();

    auto J = I.template topLeftCorner<3, 3>();
    Matrix<Scalar, 3, 1> c;
    c << I(2, 4), I(0, 5), I(1, 3);
    const auto& m = I(3, 3);

    auto vectorToSkewSymmetricSquared = [](const Matrix<Scalar, 3, 1>& a) {
      Matrix<Scalar, 3, 3> ret;
      auto a0_2 = a(0) * a(0);
      auto a1_2 = a(1) * a(1);
      auto a2_2 = a(2) * a(2);

      ret(0, 0) = -a1_2 - a2_2;
      ret(0, 1) = a(0) * a(1);
      ret(0, 2) = a(0) * a(2);

      ret(1, 0) = ret(0, 1);
      ret(1, 1) = -a0_2 - a2_2;
      ret(1, 2) = a(1) * a(2);

      ret(2, 0) = ret(0, 2);
      ret(2, 1) = ret(1, 2);
      ret(2, 2) = -a0_2 - a1_2;
      return ret;
    };

    drake::SquareTwistMatrix<Scalar> I_new;
    auto c_new = (R * c).eval();
    auto J_new = I_new.template topLeftCorner<3, 3>();

    if (m > NumTraits<Scalar>::epsilon()) {
      J_new = vectorToSkewSymmetricSquared(c_new);
      c_new.noalias() += m * p;
      J_new -= vectorToSkewSymmetricSquared(c_new);
      J_new /= m;
    } else {
      J_new.setZero();
    }
    J_new.noalias() += R * J.template selfadjointView<Lower>() * R.transpose();

    I_new.template topRightCorner<3, 3>() = vectorToSkewSymmetric(c_new);
    I_new.template bottomLeftCorner<3, 3>() =
        -I_new.template topRightCorner<3, 3>();
    I_new.template bottomRightCorner<3, 3>() =
        I.template bottomRightCorner<3, 3>();

    return I_new;
  } else {
    auto I_half_transformed = transformSpatialForce(T_current_to_new, I);
    return transformSpatialForce(T_current_to_new,
                                 I_half_transformed.transpose());
  }
}

template <typename DerivedA, typename DerivedB>
typename TransformSpatial<DerivedB>::type crossSpatialMotion(
    const Eigen::MatrixBase<DerivedA>& a,
    const Eigen::MatrixBase<DerivedB>& b) {
  typename TransformSpatial<DerivedB>::type ret(drake::kTwistSize, b.cols());
  ret.template topRows<3>() =
      -b.template topRows<3>().colwise().cross(a.template topRows<3>());
  ret.template bottomRows<3>() =
      -b.template topRows<3>().colwise().cross(a.template bottomRows<3>());
  ret.template bottomRows<3>() -=
      b.template bottomRows<3>().colwise().cross(a.template topRows<3>());
  return ret;
}

template <typename DerivedA, typename DerivedB>
typename TransformSpatial<DerivedB>::type crossSpatialForce(
    const Eigen::MatrixBase<DerivedA>& a,
    const Eigen::MatrixBase<DerivedB>& b) {
  typename TransformSpatial<DerivedB>::type ret(drake::kTwistSize, b.cols());
  ret.template topRows<3>() =
      -b.template topRows<3>().colwise().cross(a.template topRows<3>());
  ret.template topRows<3>() -=
      b.template bottomRows<3>().colwise().cross(a.template bottomRows<3>());
  ret.template bottomRows<3>() =
      -b.template bottomRows<3>().colwise().cross(a.template topRows<3>());
  return ret;
}

template <typename DerivedA, typename DerivedB>
drake::TwistMatrix<typename DerivedA::Scalar> dCrossSpatialMotion(
    const Eigen::MatrixBase<DerivedA>& a, const Eigen::MatrixBase<DerivedB>& b,
    const typename drake::math::Gradient<DerivedA, Eigen::Dynamic>::type& da,
    const typename drake::math::Gradient<DerivedB, Eigen::Dynamic>::type& db) {
  drake::TwistMatrix<typename DerivedA::Scalar> ret(drake::kTwistSize,
                                                    da.cols());
  ret.row(0) = -da.row(2) * b[1] + da.row(1) * b[2] - a[2] * db.row(1) +
               a[1] * db.row(2);
  ret.row(1) =
      da.row(2) * b[0] - da.row(0) * b[2] + a[2] * db.row(0) - a[0] * db.row(2);
  ret.row(2) = -da.row(1) * b[0] + da.row(0) * b[1] - a[1] * db.row(0) +
               a[0] * db.row(1);
  ret.row(3) = -da.row(5) * b[1] + da.row(4) * b[2] - da.row(2) * b[4] +
               da.row(1) * b[5] - a[5] * db.row(1) + a[4] * db.row(2) -
               a[2] * db.row(4) + a[1] * db.row(5);
  ret.row(4) = da.row(5) * b[0] - da.row(3) * b[2] + da.row(2) * b[3] -
               da.row(0) * b[5] + a[5] * db.row(0) - a[3] * db.row(2) +
               a[2] * db.row(3) - a[0] * db.row(5);
  ret.row(5) = -da.row(4) * b[0] + da.row(3) * b[1] - da.row(1) * b[3] +
               da.row(0) * b[4] - a[4] * db.row(0) + a[3] * db.row(1) -
               a[1] * db.row(3) + a[0] * db.row(4);
  return ret;
}

template <typename DerivedA, typename DerivedB>
drake::TwistMatrix<typename DerivedA::Scalar> dCrossSpatialForce(
    const Eigen::MatrixBase<DerivedA>& a, const Eigen::MatrixBase<DerivedB>& b,
    const typename drake::math::Gradient<DerivedA, Eigen::Dynamic>::type& da,
    const typename drake::math::Gradient<DerivedB, Eigen::Dynamic>::type& db) {
  drake::TwistMatrix<typename DerivedA::Scalar> ret(drake::kTwistSize,
                                                    da.cols());
  ret.row(0) = da.row(2) * b[1] - da.row(1) * b[2] + da.row(5) * b[4] -
               da.row(4) * b[5] + a[2] * db.row(1) - a[1] * db.row(2) +
               a[5] * db.row(4) - a[4] * db.row(5);
  ret.row(1) = -da.row(2) * b[0] + da.row(0) * b[2] - da.row(5) * b[3] +
               da.row(3) * b[5] - a[2] * db.row(0) + a[0] * db.row(2) -
               a[5] * db.row(3) + a[3] * db.row(5);
  ret.row(2) = da.row(1) * b[0] - da.row(0) * b[1] + da.row(4) * b[3] -
               da.row(3) * b[4] + a[1] * db.row(0) - a[0] * db.row(1) +
               a[4] * db.row(3) - a[3] * db.row(4);
  ret.row(3) =
      da.row(2) * b[4] - da.row(1) * b[5] + a[2] * db.row(4) - a[1] * db.row(5);
  ret.row(4) = -da.row(2) * b[3] + da.row(0) * b[5] - a[2] * db.row(3) +
               a[0] * db.row(5);
  ret.row(5) =
      da.row(1) * b[3] - da.row(0) * b[4] + a[1] * db.row(3) - a[0] * db.row(4);
  ret = -ret;
  return ret;
}

/*
 * spatial transform gradient methods
 */
template <typename DerivedQdotToV>
struct DHomogTrans {
  typedef typename Eigen::Matrix<typename DerivedQdotToV::Scalar,
                                 HOMOGENEOUS_TRANSFORM_SIZE,
                                 DerivedQdotToV::ColsAtCompileTime> type;
};

template <typename DerivedS, typename DerivedQdotToV>
typename DHomogTrans<DerivedQdotToV>::type dHomogTrans(
    const Eigen::Transform<typename DerivedQdotToV::Scalar, 3, Eigen::Isometry>&
        T,
    const Eigen::MatrixBase<DerivedS>& S,
    const Eigen::MatrixBase<DerivedQdotToV>& qdot_to_v) {
  const int nq_at_compile_time = DerivedQdotToV::ColsAtCompileTime;
  typename DerivedQdotToV::Index nq = qdot_to_v.cols();
  auto qdot_to_twist = (S * qdot_to_v).eval();

  const int numel = HOMOGENEOUS_TRANSFORM_SIZE;
  Eigen::Matrix<typename DerivedQdotToV::Scalar, numel, nq_at_compile_time> ret(
      numel, nq);

  const auto& Rx = T.linear().col(0);
  const auto& Ry = T.linear().col(1);
  const auto& Rz = T.linear().col(2);

  const auto& qdot_to_omega_x = qdot_to_twist.row(0);
  const auto& qdot_to_omega_y = qdot_to_twist.row(1);
  const auto& qdot_to_omega_z = qdot_to_twist.row(2);

  ret.template middleRows<3>(0) = -Rz * qdot_to_omega_y + Ry * qdot_to_omega_z;
  ret.row(3).setZero();

  ret.template middleRows<3>(4) = Rz * qdot_to_omega_x - Rx * qdot_to_omega_z;
  ret.row(7).setZero();

  ret.template middleRows<3>(8) = -Ry * qdot_to_omega_x + Rx * qdot_to_omega_y;
  ret.row(11).setZero();

  ret.template middleRows<3>(12) = T.linear() * qdot_to_twist.bottomRows(3);
  ret.row(15).setZero();

  return ret;
}

template <typename DerivedDT>
typename DHomogTrans<DerivedDT>::type dHomogTransInv(
    const Eigen::Transform<typename DerivedDT::Scalar, 3, Eigen::Isometry>& T,
    const Eigen::MatrixBase<DerivedDT>& dT) {
  typename DerivedDT::Index nq = dT.cols();

  const auto& R = T.linear();
  const auto& p = T.translation();

  std::array<int, 3> rows = {{0, 1, 2}};
  std::array<int, 3> R_cols = {{0, 1, 2}};
  std::array<int, 1> p_cols = {{3}};

  auto dR = getSubMatrixGradient<Eigen::Dynamic>(dT, rows, R_cols, T.Rows);
  auto dp = getSubMatrixGradient<Eigen::Dynamic>(dT, rows, p_cols, T.Rows);

  auto dinvT_R = transposeGrad(dR, R.rows());
  auto dinvT_p = (-R.transpose() * dp - matGradMult(dinvT_R, p)).eval();

  const int numel = HOMOGENEOUS_TRANSFORM_SIZE;
  Eigen::Matrix<typename DerivedDT::Scalar, numel, DerivedDT::ColsAtCompileTime>
      ret(numel, nq);
  setSubMatrixGradient<Eigen::Dynamic>(ret, dinvT_R, rows, R_cols, T.Rows);
  setSubMatrixGradient<Eigen::Dynamic>(ret, dinvT_p, rows, p_cols, T.Rows);

  // zero out gradient of elements in last row:
  const int last_row = 3;
  for (int col = 0; col < T.HDim; col++) {
    ret.row(last_row + col * T.Rows).setZero();
  }

  return ret;
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 1> flipExpmap(
    const Eigen::MatrixBase<Derived>& expmap) {
  using namespace Eigen;
  typedef typename Derived::Scalar Scalar;
  static_assert(
      Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 1,
      "Wrong size.");

  Scalar expmap_norm = expmap.norm();
  bool is_degenerate = (expmap_norm < std::numeric_limits<double>::epsilon());
  Eigen::Matrix<Scalar, 3, 1> ret = expmap;
  if (!is_degenerate) ret -= expmap / expmap_norm * 2 * M_PI;

  return ret;
}

template <typename Derived1, typename Derived2>
Eigen::Matrix<typename Derived1::Scalar, 3, 1> unwrapExpmap(
    const Eigen::MatrixBase<Derived1>& expmap1,
    const Eigen::MatrixBase<Derived2>& expmap2) {
  using namespace Eigen;
  static_assert(
      Derived1::RowsAtCompileTime == 3 && Derived1::ColsAtCompileTime == 1,
      "Wrong size.");
  static_assert(
      Derived2::RowsAtCompileTime == 3 && Derived2::ColsAtCompileTime == 1,
      "Wrong size.");
  static_assert(
      std::is_same<typename Derived1::Scalar, typename Derived2::Scalar>::value,
      "Scalar types don't match.");
  typedef typename Derived1::Scalar Scalar;
  typedef typename NumTraits<Scalar>::Real Real;

  auto expmap2_flip = flipExpmap(expmap2);
  Real distance1 = (expmap1 - expmap2).squaredNorm();
  Real distance2 = (expmap1 - expmap2_flip).squaredNorm();
  if (distance1 > distance2) {
    return expmap2_flip;
  } else {
    return expmap2;
  }
}
