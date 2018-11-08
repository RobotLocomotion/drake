/// @file
/// THIS FILE IS DEPRECATED.
/// Its contents are moving into drake/math.

#pragma once

#include <cmath>
#include <cstring>
#include <random>

#include <Eigen/Dense>

#include "drake/common/constants.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/math/cross_product.h"
#include "drake/math/gradient.h"
#include "drake/math/gradient_util.h"
#include "drake/math/normalize_vector.h"
#include "drake/math/quaternion.h"
#include "drake/math/rotation_conversion_gradient.h"

// TODO(jwnimmer-tri): Clean up function naming and other styleguide defects.

double angleDiff(double phi1, double phi2);

int rotationRepresentationSize(int rotation_type);

/*
 * angular velocity conversion functions
 */
template <typename DerivedQ, typename DerivedM, typename DerivedDM>
void angularvel2quatdotMatrix(const Eigen::MatrixBase<DerivedQ>& q,
                              Eigen::MatrixBase<DerivedM>& M,
                              Eigen::MatrixBase<DerivedDM>* dM = nullptr) {
  // note: not normalizing to match MATLAB implementation
  using Scalar = typename DerivedQ::Scalar;
  M.resize(drake::kQuaternionSize, drake::kSpaceDimension);
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
  phi.resize(drake::kRpySize, drake::kSpaceDimension);

  typedef typename DerivedRPY::Scalar Scalar;
  Scalar p = rpy(1);
  Scalar y = rpy(2);

  // NOLINTNEXTLINE(build/namespaces): Needed for ADL.
  using namespace std;
  Scalar sy = sin(y);
  Scalar cy = cos(y);
  Scalar sp = sin(p);
  Scalar cp = cos(p);
  Scalar tp = sp / cp;

  phi << cy / cp, sy / cp, Scalar(0), -sy, cy, Scalar(0), cy * tp, tp * sy,
      Scalar(1);
  if (dphi) {
    dphi->resize(phi.size(), drake::kRpySize);
    Scalar sp2 = sp * sp;
    Scalar cp2 = cp * cp;
    (*dphi) << Scalar(0), (cy * sp) / cp2, -sy / cp, Scalar(0), Scalar(0), -cy,
        Scalar(0), cy + (cy * sp2) / cp2, -(sp * sy) / cp, Scalar(0),
        (sp * sy) / cp2, cy / cp, Scalar(0), Scalar(0), -sy, Scalar(0),
        sy + (sp2 * sy) / cp2, (cy * sp) / cp, Scalar(0), Scalar(0), Scalar(0),
        Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0), Scalar(0);

    if (ddphi) {
      ddphi->resize(dphi->size(), drake::kRpySize);
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
    typename drake::math::Gradient<DerivedE, drake::kRpySize, 1>::type* dE =
        nullptr) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<DerivedRPY>,
                                           drake::kRpySize);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(
      Eigen::MatrixBase<DerivedE>, drake::kSpaceDimension, drake::kRpySize);
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
                                           drake::kQuaternionSize);
  typedef typename Derived::Scalar Scalar;
  auto qtilde = q.normalized();
  Eigen::Matrix<Scalar, 3, 4> ret;
  ret << -qtilde(1), qtilde(0), -qtilde(3), qtilde(2), -qtilde(2), qtilde(3),
      qtilde(0), -qtilde(1), -qtilde(3), -qtilde(2), qtilde(1), qtilde(0);
  ret *= Scalar(2);
  return ret;
}

/*
 * spatial transform functions
 */

// This function is only applicable to input matrices with columns less than or
// equal to 6 and will assert if the number of columns is greater than 6.
template <typename DerivedM>
decltype(auto) transformSpatialMotion(
    const Eigen::Transform<typename DerivedM::Scalar, 3, Eigen::Isometry>& T,
    const Eigen::MatrixBase<DerivedM>& M) {
  DRAKE_ASSERT(M.cols() <= 6);
  Eigen::Matrix<typename DerivedM::Scalar, drake::kTwistSize,
                DerivedM::ColsAtCompileTime, 0, drake::kTwistSize,
                DerivedM::ColsAtCompileTime == Eigen::Dynamic
                  ? 6 : DerivedM::ColsAtCompileTime>
      ret(drake::kTwistSize, M.cols());
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

  auto dR = drake::math::getSubMatrixGradient<Eigen::Dynamic>(
      dT, rows, R_cols, T.Rows);
  auto dp = drake::math::getSubMatrixGradient<Eigen::Dynamic>(
      dT, rows, p_cols, T.Rows);

  typename drake::math::Gradient<DerivedX, DerivedDX::ColsAtCompileTime,
                                 1>::type ret(X.size(), nq);
  std::array<int, 3> Xomega_rows = {{0, 1, 2}};
  std::array<int, 3> Xv_rows = {{3, 4, 5}};
  for (int col = 0; col < X.cols(); col++) {
    auto Xomega_col = X.template block<3, 1>(0, col);
    auto Xv_col = X.template block<3, 1>(3, col);

    auto RXomega_col = (R * Xomega_col).eval();

    std::array<int, 1> col_array = {{col}};
    auto dXomega_col = drake::math::getSubMatrixGradient<Eigen::Dynamic>(
        dX, Xomega_rows, col_array, X.rows());
    auto dXv_col = drake::math::getSubMatrixGradient<Eigen::Dynamic>(
        dX, Xv_rows, col_array, X.rows());

    auto domega_part_col =
        (R * dXomega_col + drake::math::matGradMult(dR, Xomega_col)).eval();
    auto dv_part_col =
        (R * dXv_col + drake::math::matGradMult(dR, Xv_col)).eval();
    dv_part_col += dp.colwise().cross(RXomega_col);
    dv_part_col -= domega_part_col.colwise().cross(p);

    drake::math::setSubMatrixGradient<Eigen::Dynamic>(
        ret, domega_part_col, Xomega_rows, col_array, X.rows());
    drake::math::setSubMatrixGradient<Eigen::Dynamic>(
        ret, dv_part_col, Xv_rows, col_array, X.rows());
  }
  return ret;
}

template <typename DerivedF>
decltype(auto) transformSpatialForce(
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

  auto dR = drake::math::getSubMatrixGradient<Eigen::Dynamic>(
      dT, rows, R_cols, T.Rows);
  auto dp = drake::math::getSubMatrixGradient<Eigen::Dynamic>(
      dT, rows, p_cols, T.Rows);

  typename drake::math::Gradient<DerivedX, DerivedDX::ColsAtCompileTime>::type
      ret(X.size(), nq);
  std::array<int, 3> Xomega_rows = {{0, 1, 2}};
  std::array<int, 3> Xv_rows = {{3, 4, 5}};
  for (int col = 0; col < X.cols(); col++) {
    auto Xomega_col = X.template block<3, 1>(0, col);
    auto Xv_col = X.template block<3, 1>(3, col);

    auto RXv_col = (R * Xv_col).eval();

    std::array<int, 1> col_array = {{col}};
    auto dXomega_col = drake::math::getSubMatrixGradient<Eigen::Dynamic>(
        dX, Xomega_rows, col_array, X.rows());
    auto dXv_col = drake::math::getSubMatrixGradient<Eigen::Dynamic>(
        dX, Xv_rows, col_array, X.rows());

    auto domega_part_col = (R * dXomega_col).eval();
    domega_part_col += drake::math::matGradMult(dR, Xomega_col);
    auto dv_part_col = (R * dXv_col).eval();
    dv_part_col += drake::math::matGradMult(dR, Xv_col);
    domega_part_col += dp.colwise().cross(RXv_col);
    domega_part_col -= dv_part_col.colwise().cross(p);

    drake::math::setSubMatrixGradient<Eigen::Dynamic>(
        ret, domega_part_col, Xomega_rows, col_array, X.rows());
    drake::math::setSubMatrixGradient<Eigen::Dynamic>(
        ret, dv_part_col, Xv_rows, col_array, X.rows());
  }
  return ret;
}

template <typename DerivedI>
bool isRegularInertiaMatrix(const Eigen::MatrixBase<DerivedI>& I) {
  using Scalar = typename DerivedI::Scalar;
  bool ret = true;

  auto J = I.template topLeftCorner<3, 3>();
  auto cross_part_1 = I.template topRightCorner<3, 3>();
  auto cross_part_2 = I.template bottomLeftCorner<3, 3>();
  const auto& m = I(3, 3);
  ret = ret && (J - J.transpose()).isZero();  // J symmetric
  ret = ret &&
        (m * Eigen::Matrix<Scalar, 3, 3>::Identity() -
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
    const Eigen::Transform<typename DerivedI::Scalar, drake::kSpaceDimension,
                           Eigen::Isometry>& T_current_to_new,
    const Eigen::MatrixBase<DerivedI>& I) {
  using Scalar = typename DerivedI::Scalar;

  if (isRegularInertiaMatrix(I)) {
    // this check is necessary to support the nonstandard inertia matrices
    // resulting from added masses

    // TODO(tkoolen): SpatialInertiaMatrix class that keeps track of whether
    // matrix is regular or not
    const auto& R = T_current_to_new.linear();
    const auto& p = T_current_to_new.translation();

    auto J = I.template topLeftCorner<3, 3>();
    Eigen::Matrix<Scalar, 3, 1> c;
    c << I(2, 4), I(0, 5), I(1, 3);
    const auto& m = I(3, 3);

    auto vectorToSkewSymmetricSquared =
        [](const Eigen::Matrix<Scalar, 3, 1>& a) {
          Eigen::Matrix<Scalar, 3, 3> ret;
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

    if (m > Eigen::NumTraits<Scalar>::epsilon()) {
      J_new = vectorToSkewSymmetricSquared(c_new);
      c_new.noalias() += m * p;
      J_new -= vectorToSkewSymmetricSquared(c_new);
      J_new /= m;
    } else {
      J_new.setZero();
    }
    J_new.noalias() +=
        R * J.template selfadjointView<Eigen::Lower>() * R.transpose();

    I_new.template topRightCorner<3, 3>() =
        drake::math::VectorToSkewSymmetric(c_new);
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
decltype(auto) crossSpatialMotion(
    const Eigen::MatrixBase<DerivedA>& a,
    const Eigen::MatrixBase<DerivedB>& b) {
  Eigen::Matrix<typename DerivedB::Scalar, drake::kTwistSize,
                DerivedB::ColsAtCompileTime> ret(drake::kTwistSize, b.cols());
  ret.template topRows<3>() =
      -b.template topRows<3>().colwise().cross(a.template topRows<3>());
  ret.template bottomRows<3>() =
      -b.template topRows<3>().colwise().cross(a.template bottomRows<3>());
  ret.template bottomRows<3>() -=
      b.template bottomRows<3>().colwise().cross(a.template topRows<3>());
  return ret;
}

template <typename DerivedA, typename DerivedB>
decltype(auto) crossSpatialForce(
    const Eigen::MatrixBase<DerivedA>& a,
    const Eigen::MatrixBase<DerivedB>& b) {
  Eigen::Matrix<typename DerivedB::Scalar, drake::kTwistSize,
                DerivedB::ColsAtCompileTime> ret(drake::kTwistSize, b.cols());
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
                                 drake::kHomogeneousTransformSize,
                                 DerivedQdotToV::ColsAtCompileTime>
      type;
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

  const int numel = drake::kHomogeneousTransformSize;
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

  auto dR = drake::math::getSubMatrixGradient<Eigen::Dynamic>(
      dT, rows, R_cols, T.Rows);
  auto dp = drake::math::getSubMatrixGradient<Eigen::Dynamic>(
      dT, rows, p_cols, T.Rows);

  auto dinvT_R = drake::math::transposeGrad(dR, R.rows());
  auto dinvT_p =
      (-R.transpose() * dp - drake::math::matGradMult(dinvT_R, p)).eval();

  const int numel = drake::kHomogeneousTransformSize;
  Eigen::Matrix<typename DerivedDT::Scalar, numel, DerivedDT::ColsAtCompileTime>
      ret(numel, nq);
  drake::math::setSubMatrixGradient<Eigen::Dynamic>(
      ret, dinvT_R, rows, R_cols, T.Rows);
  drake::math::setSubMatrixGradient<Eigen::Dynamic>(
      ret, dinvT_p, rows, p_cols, T.Rows);

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
  typedef typename Eigen::NumTraits<Scalar>::Real Real;

  auto expmap2_flip = flipExpmap(expmap2);
  Real distance1 = (expmap1 - expmap2).squaredNorm();
  Real distance2 = (expmap1 - expmap2_flip).squaredNorm();
  if (distance1 > distance2) {
    return expmap2_flip;
  } else {
    return expmap2;
  }
}
