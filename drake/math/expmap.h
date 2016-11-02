/// @file
/// Utilities for arithmetic on exponential maps.

#pragma once

#include <cmath>

#include <Eigen/Dense>

#include "drake/common/drake_assert.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/autodiff_overloads.h"

namespace drake {
namespace math {

namespace internal {
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 1> expmap2quatNonDegenerate(
    const Eigen::MatrixBase<Derived>& v,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    typename Derived::Scalar& theta_squared) {
  using namespace std;  // NOLINT(build/namespaces)
  typedef typename Derived::Scalar Scalar;
  static_assert(
      Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 1,
      "Wrong size.");

  Eigen::Matrix<Scalar, 4, 1> q;

  Scalar theta = sqrt(theta_squared);
  Scalar arg = theta / Scalar(2);
  q(0) = cos(arg);
  q.template bottomRows<3>() = v;
  q.template bottomRows<3>() *= sin(arg) / theta;

  return q;
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 1> expmap2quatDegenerate(
    const Eigen::MatrixBase<Derived>& v,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    typename Derived::Scalar& theta_squared) {
  typedef typename Derived::Scalar Scalar;
  static_assert(
      Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 1,
      "Wrong size.");

  Eigen::Matrix<Scalar, 4, 1> q;

  q(0) = -theta_squared / 8.0 + 1.0;
  q.template bottomRows<3>() = v;
  q.template bottomRows<3>() *=
      (theta_squared * 8.0E1 - 1.92E3) * (-2.604166666666667E-4);

  return q;
}
}  // namespace internal

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 4, 1> expmap2quat(
    const Eigen::MatrixBase<Derived>& v) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Eigen::MatrixBase<Derived>, 3);
  typedef typename Derived::Scalar Scalar;
  Scalar theta_squared = v.squaredNorm();
  if (theta_squared < pow(Eigen::NumTraits<Scalar>::epsilon(), 0.5)) {
    return internal::expmap2quatDegenerate(v, theta_squared);
  } else {
    return internal::expmap2quatNonDegenerate(v, theta_squared);
  }
}

template <typename DerivedQ>
Eigen::Matrix<typename DerivedQ::Scalar, 3, 1> quat2expmap(
    const Eigen::MatrixBase<DerivedQ>& q) {
  typedef typename DerivedQ::Scalar Scalar;
  static_assert(
      DerivedQ::RowsAtCompileTime == 4 && DerivedQ::ColsAtCompileTime == 1,
      "Wrong size.");

  Scalar t = sqrt(Scalar(1) - q(0) * q(0));
  bool is_degenerate = (t * t < Eigen::NumTraits<Scalar>::epsilon());
  Scalar s(2);
  if (!is_degenerate) s *= acos(q(0)) / t;
  return s * q.template tail<3>();
}

template <typename Derived1, typename Derived2>
Eigen::Matrix<typename Derived1::Scalar, 3, 1> closestExpmap(
    const Eigen::MatrixBase<Derived1>& expmap1,
    const Eigen::MatrixBase<Derived2>& expmap2) {
  // NOLINTNEXTLINE(build/namespaces)
  using namespace std;  // required for ADL of floor() and round().
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

  Real expmap1_norm = expmap1.norm();
  Real expmap2_norm = expmap2.norm();
  Eigen::Matrix<Scalar, 3, 1> ret;
  if (expmap2_norm < Eigen::NumTraits<Scalar>::epsilon()) {
    if (expmap1_norm > Eigen::NumTraits<Scalar>::epsilon()) {
      auto expmap1_axis = (expmap1 / expmap1_norm).eval();
      auto expmap1_round = round(expmap1_norm / (2 * M_PI));
      return expmap1_axis * expmap1_round * 2 * M_PI;
    } else {
      return expmap2;
    }
  } else {
    auto expmap2_axis = (expmap2 / expmap2_norm).eval();
    auto expmap2_closest_k =
        ((expmap2_axis.transpose() * expmap1).value() - expmap2_norm) /
        (2 * M_PI);
    auto expmap2_closest_k1 = floor(expmap2_closest_k);
    auto expmap2_closest_k2 = expmap2_closest_k1 + 1.0;
    auto expmap2_closest1 =
        (expmap2 + 2 * expmap2_closest_k1 * M_PI * expmap2_axis).eval();
    auto expmap2_closest2 =
        (expmap2 + 2 * expmap2_closest_k2 * M_PI * expmap2_axis).eval();
    if ((expmap2_closest1 - expmap1).norm() <
        (expmap2_closest2 - expmap1).norm()) {
      return expmap2_closest1;
    } else {
      return expmap2_closest2;
    }
  }
}

template <typename DerivedQ, typename DerivedE>
void quat2expmapSequence(const Eigen::MatrixBase<DerivedQ>& quat,
                         const Eigen::MatrixBase<DerivedQ>& quat_dot,
                         // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                         Eigen::MatrixBase<DerivedE>& expmap,
                         // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                         Eigen::MatrixBase<DerivedE>& expmap_dot) {
  static_assert(DerivedQ::RowsAtCompileTime == 4, "Wrong size.");
  static_assert(DerivedE::RowsAtCompileTime == 3, "Wrong size.");
  static_assert(
      std::is_same<typename DerivedQ::Scalar, typename DerivedE::Scalar>::value,
      "Scalar types don't match.");
  typedef typename DerivedQ::Scalar Scalar;

  DRAKE_ASSERT(quat.cols() == quat_dot.cols() &&
               "number of columns of quat doesn't match quat_dot");
  Eigen::Index N = quat.cols();

  typedef Eigen::AutoDiffScalar<Eigen::Matrix<Scalar, 1, 1>> ADScalar;
  auto quat_autodiff = quat.template cast<ADScalar>().eval();
  for (int i = 0; i < quat.size(); i++) {
    quat_autodiff(i).derivatives()(0) = quat_dot(i);
  }

  expmap.resize(3, N);
  expmap_dot.resize(3, N);
  Eigen::Matrix<ADScalar, 3, 1> expmap_autodiff_previous;
  for (int i = 0; i < N; i++) {
    auto expmap_autodiff = quat2expmap(quat_autodiff.col(i));
    if (i >= 1) {
      expmap_autodiff =
          closestExpmap(expmap_autodiff_previous, expmap_autodiff);
    }
    expmap.col(i) = autoDiffToValueMatrix(expmap_autodiff);
    expmap_dot.col(i) = autoDiffToGradientMatrix(expmap_autodiff);
    expmap_autodiff_previous = expmap_autodiff;
  }
}

}  // namespace math
}  // namespace drake
