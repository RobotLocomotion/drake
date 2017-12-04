#pragma once

// This file is a modification of Eigen-3.3.3's AutoDiffScalar.h file which is
// available at
// https://bitbucket.org/eigen/eigen/raw/67e894c6cd8f5f1f604b27d37ed47fdf012674ff/unsupported/Eigen/src/AutoDiff/AutoDiffScalar.h
//
// Copyright (C) 2009 Gael Guennebaud <gael.guennebaud@inria.fr>
// Copyright (C) 2017 Drake Authors
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef DRAKE_COMMON_AUTODIFF_HEADER
// TODO(soonho-tri): Change to #error.
#warning Do not directly include this file. Include "drake/common/autodiff.h".
#endif

#include <cmath>
#include <ostream>

#include <Eigen/Dense>

namespace Eigen {

#if !defined(DRAKE_DOXYGEN_CXX)
// Explicit template specializations of Eigen::AutoDiffScalar for VectorXd.
//
// AutoDiffScalar tries to call internal::make_coherent to promote empty
// derivatives. However, it fails to do the promotion when an operand is an
// expression tree (i.e. CwiseBinaryOp). Our solution is to provide special
// overloading for VectorXd and change the return types of its operators. With
// this change, the operators evaluate terms immediately and return an
// AutoDiffScalar<VectorXd> instead of expression trees (such as CwiseBinaryOp).
// This ensures internal::make_coherent does the correct operation (resizing and
// setting-to-zero) when needed.
//
// We also provide overloading of math functions for AutoDiffScalar<VectorXd>
// which return AutoDiffScalar<VectorXd> instead of an expression tree.
//
// See https://github.com/RobotLocomotion/drake/issues/6944 for more
// information. See also drake/common/autodiff_overloads.h.
//
// TODO(soonho-tri): Next time when we upgrade Eigen, please check if we still
// need these specializations.
template <>
class AutoDiffScalar<VectorXd>
    : public internal::auto_diff_special_op<VectorXd, false> {
 public:
  typedef internal::auto_diff_special_op<VectorXd, false> Base;
  typedef typename internal::remove_all<VectorXd>::type DerType;
  typedef typename internal::traits<DerType>::Scalar Scalar;
  typedef typename NumTraits<Scalar>::Real Real;

  using Base::operator+;
  using Base::operator*;

  AutoDiffScalar() {}

  AutoDiffScalar(const Scalar& value, int nbDer, int derNumber)
      : m_value(value), m_derivatives(DerType::Zero(nbDer)) {
    m_derivatives.coeffRef(derNumber) = Scalar(1);
  }

  // NOLINTNEXTLINE(runtime/explicit): Code from Eigen.
  AutoDiffScalar(const Real& value) : m_value(value) {
    if (m_derivatives.size() > 0) m_derivatives.setZero();
  }

  AutoDiffScalar(const Scalar& value, const DerType& der)
      : m_value(value), m_derivatives(der) {}

  template <typename OtherDerType>
  AutoDiffScalar(
      const AutoDiffScalar<OtherDerType>& other
#ifndef EIGEN_PARSED_BY_DOXYGEN
      ,
      typename internal::enable_if<
          internal::is_same<
              Scalar, typename internal::traits<typename internal::remove_all<
                          OtherDerType>::type>::Scalar>::value,
          void*>::type = 0
#endif
      )
      : m_value(other.value()), m_derivatives(other.derivatives()) {
  }

  friend std::ostream& operator<<(std::ostream& s, const AutoDiffScalar& a) {
    return s << a.value();
  }

  AutoDiffScalar(const AutoDiffScalar& other)
      : m_value(other.value()), m_derivatives(other.derivatives()) {}

  template <typename OtherDerType>
  inline AutoDiffScalar& operator=(const AutoDiffScalar<OtherDerType>& other) {
    m_value = other.value();
    m_derivatives = other.derivatives();
    return *this;
  }

  inline AutoDiffScalar& operator=(const AutoDiffScalar& other) {
    m_value = other.value();
    m_derivatives = other.derivatives();
    return *this;
  }

  inline AutoDiffScalar& operator=(const Scalar& other) {
    m_value = other;
    if (m_derivatives.size() > 0) m_derivatives.setZero();
    return *this;
  }

  inline const Scalar& value() const { return m_value; }
  inline Scalar& value() { return m_value; }

  inline const DerType& derivatives() const { return m_derivatives; }
  inline DerType& derivatives() { return m_derivatives; }

  inline bool operator<(const Scalar& other) const { return m_value < other; }
  inline bool operator<=(const Scalar& other) const { return m_value <= other; }
  inline bool operator>(const Scalar& other) const { return m_value > other; }
  inline bool operator>=(const Scalar& other) const { return m_value >= other; }
  inline bool operator==(const Scalar& other) const { return m_value == other; }
  inline bool operator!=(const Scalar& other) const { return m_value != other; }

  friend inline bool operator<(const Scalar& a, const AutoDiffScalar& b) {
    return a < b.value();
  }
  friend inline bool operator<=(const Scalar& a, const AutoDiffScalar& b) {
    return a <= b.value();
  }
  friend inline bool operator>(const Scalar& a, const AutoDiffScalar& b) {
    return a > b.value();
  }
  friend inline bool operator>=(const Scalar& a, const AutoDiffScalar& b) {
    return a >= b.value();
  }
  friend inline bool operator==(const Scalar& a, const AutoDiffScalar& b) {
    return a == b.value();
  }
  friend inline bool operator!=(const Scalar& a, const AutoDiffScalar& b) {
    return a != b.value();
  }

  template <typename OtherDerType>
  inline bool operator<(const AutoDiffScalar<OtherDerType>& b) const {
    return m_value < b.value();
  }
  template <typename OtherDerType>
  inline bool operator<=(const AutoDiffScalar<OtherDerType>& b) const {
    return m_value <= b.value();
  }
  template <typename OtherDerType>
  inline bool operator>(const AutoDiffScalar<OtherDerType>& b) const {
    return m_value > b.value();
  }
  template <typename OtherDerType>
  inline bool operator>=(const AutoDiffScalar<OtherDerType>& b) const {
    return m_value >= b.value();
  }
  template <typename OtherDerType>
  inline bool operator==(const AutoDiffScalar<OtherDerType>& b) const {
    return m_value == b.value();
  }
  template <typename OtherDerType>
  inline bool operator!=(const AutoDiffScalar<OtherDerType>& b) const {
    return m_value != b.value();
  }

  inline const AutoDiffScalar<DerType> operator+(const Scalar& other) const {
    return AutoDiffScalar<DerType>(m_value + other, m_derivatives);
  }

  friend inline const AutoDiffScalar<DerType> operator+(
      const Scalar& a, const AutoDiffScalar& b) {
    return AutoDiffScalar<DerType>(a + b.value(), b.derivatives());
  }

  inline AutoDiffScalar& operator+=(const Scalar& other) {
    value() += other;
    return *this;
  }

  template <typename OtherDerType>
  inline const AutoDiffScalar<DerType> operator+(
      const AutoDiffScalar<OtherDerType>& other) const {
    internal::make_coherent(m_derivatives, other.derivatives());
    return AutoDiffScalar<DerType>(m_value + other.value(),
                                   m_derivatives + other.derivatives());
  }

  template <typename OtherDerType>
  inline AutoDiffScalar& operator+=(const AutoDiffScalar<OtherDerType>& other) {
    (*this) = (*this) + other;
    return *this;
  }

  inline const AutoDiffScalar<DerType> operator-(const Scalar& b) const {
    return AutoDiffScalar<DerType>(m_value - b, m_derivatives);
  }

  friend inline const AutoDiffScalar<DerType> operator-(
      const Scalar& a, const AutoDiffScalar& b) {
    return AutoDiffScalar<DerType>(a - b.value(), -b.derivatives());
  }

  inline AutoDiffScalar& operator-=(const Scalar& other) {
    value() -= other;
    return *this;
  }

  template <typename OtherDerType>
  inline const AutoDiffScalar<DerType> operator-(
      const AutoDiffScalar<OtherDerType>& other) const {
    internal::make_coherent(m_derivatives, other.derivatives());
    return AutoDiffScalar<DerType>(m_value - other.value(),
                                   m_derivatives - other.derivatives());
  }

  template <typename OtherDerType>
  inline AutoDiffScalar& operator-=(const AutoDiffScalar<OtherDerType>& other) {
    *this = *this - other;
    return *this;
  }

  inline const AutoDiffScalar<DerType> operator-() const {
    return AutoDiffScalar<DerType>(-m_value, -m_derivatives);
  }

  inline const AutoDiffScalar<DerType> operator*(const Scalar& other) const {
    return MakeAutoDiffScalar(m_value * other, m_derivatives * other);
  }

  friend inline const AutoDiffScalar<DerType> operator*(
      const Scalar& other, const AutoDiffScalar& a) {
    return MakeAutoDiffScalar(a.value() * other, a.derivatives() * other);
  }

  inline const AutoDiffScalar<DerType> operator/(const Scalar& other) const {
    return MakeAutoDiffScalar(m_value / other,
                              (m_derivatives * (Scalar(1) / other)));
  }

  friend inline const AutoDiffScalar<DerType> operator/(
      const Scalar& other, const AutoDiffScalar& a) {
    return MakeAutoDiffScalar(
        other / a.value(),
        a.derivatives() * (Scalar(-other) / (a.value() * a.value())));
  }

  template <typename OtherDerType>
  inline const AutoDiffScalar<DerType> operator/(
      const AutoDiffScalar<OtherDerType>& other) const {
    internal::make_coherent(m_derivatives, other.derivatives());
    return MakeAutoDiffScalar(
        m_value / other.value(),
        ((m_derivatives * other.value()) - (other.derivatives() * m_value)) *
            (Scalar(1) / (other.value() * other.value())));
  }

  template <typename OtherDerType>
  inline const AutoDiffScalar<DerType> operator*(
      const AutoDiffScalar<OtherDerType>& other) const {
    internal::make_coherent(m_derivatives, other.derivatives());
    return MakeAutoDiffScalar(
        m_value * other.value(),
        (m_derivatives * other.value()) + (other.derivatives() * m_value));
  }

  inline AutoDiffScalar& operator*=(const Scalar& other) {
    *this = *this * other;
    return *this;
  }

  template <typename OtherDerType>
  inline AutoDiffScalar& operator*=(const AutoDiffScalar<OtherDerType>& other) {
    *this = *this * other;
    return *this;
  }

  inline AutoDiffScalar& operator/=(const Scalar& other) {
    *this = *this / other;
    return *this;
  }

  template <typename OtherDerType>
  inline AutoDiffScalar& operator/=(const AutoDiffScalar<OtherDerType>& other) {
    *this = *this / other;
    return *this;
  }

 protected:
  Scalar m_value;
  DerType m_derivatives;
};

#define DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(FUNC, CODE) \
  inline const AutoDiffScalar<VectorXd> FUNC(                   \
      const AutoDiffScalar<VectorXd>& x) {                      \
    EIGEN_UNUSED typedef double Scalar;                         \
    CODE;                                                       \
  }

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    abs, using std::abs; return Eigen::MakeAutoDiffScalar(
        abs(x.value()), x.derivatives() * (x.value() < 0 ? -1 : 1));)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    abs2, using numext::abs2; return Eigen::MakeAutoDiffScalar(
        abs2(x.value()), x.derivatives() * (Scalar(2) * x.value()));)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    sqrt, using std::sqrt; Scalar sqrtx = sqrt(x.value());
    return Eigen::MakeAutoDiffScalar(sqrtx,
                                     x.derivatives() * (Scalar(0.5) / sqrtx));)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    cos, using std::cos; using std::sin;
    return Eigen::MakeAutoDiffScalar(cos(x.value()),
                                     x.derivatives() * (-sin(x.value())));)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    sin, using std::sin; using std::cos;
    return Eigen::MakeAutoDiffScalar(sin(x.value()),
                                     x.derivatives() * cos(x.value()));)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    exp, using std::exp; Scalar expx = exp(x.value());
    return Eigen::MakeAutoDiffScalar(expx, x.derivatives() * expx);)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    log, using std::log; return Eigen::MakeAutoDiffScalar(
        log(x.value()), x.derivatives() * (Scalar(1) / x.value()));)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    tan, using std::tan; using std::cos; return Eigen::MakeAutoDiffScalar(
        tan(x.value()),
        x.derivatives() * (Scalar(1) / numext::abs2(cos(x.value()))));)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    asin, using std::sqrt; using std::asin; return Eigen::MakeAutoDiffScalar(
        asin(x.value()),
        x.derivatives() * (Scalar(1) / sqrt(1 - numext::abs2(x.value()))));)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    acos, using std::sqrt; using std::acos; return Eigen::MakeAutoDiffScalar(
        acos(x.value()),
        x.derivatives() * (Scalar(-1) / sqrt(1 - numext::abs2(x.value()))));)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    tanh, using std::cosh; using std::tanh; return Eigen::MakeAutoDiffScalar(
        tanh(x.value()),
        x.derivatives() * (Scalar(1) / numext::abs2(cosh(x.value()))));)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    sinh, using std::sinh; using std::cosh;
    return Eigen::MakeAutoDiffScalar(sinh(x.value()),
                                     x.derivatives() * cosh(x.value()));)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    cosh, using std::sinh; using std::cosh;
    return Eigen::MakeAutoDiffScalar(cosh(x.value()),
                                     x.derivatives() * sinh(x.value()));)

#undef DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY

// We have this specialization here because the Eigen-3.3.3's atan2
// implementation for AutoDiffScalar does not call `make_coherent` function.
inline const AutoDiffScalar<VectorXd> atan2(const AutoDiffScalar<VectorXd>& a,
                                            const AutoDiffScalar<VectorXd>& b) {
  using std::atan2;
  typedef double Scalar;
  typedef AutoDiffScalar<Matrix<Scalar, Dynamic, 1>> PlainADS;

  internal::make_coherent(a.derivatives(), b.derivatives());

  PlainADS ret;
  ret.value() = atan2(a.value(), b.value());

  Scalar squared_hypot = a.value() * a.value() + b.value() * b.value();

  // if (squared_hypot==0) the derivation is undefined and the following results
  // in a NaN:
  ret.derivatives() =
      (a.derivatives() * b.value() - a.value() * b.derivatives()) /
      squared_hypot;

  return ret;
}

// We have this specialization here because the Eigen-3.3.3's implementation of
// min for AutoDiffScalar does not call `make_coherent` function.
//
// For example, `min(x, y) + x` gives a runtime error if x.value() < y.value()
// but x's derivatives are not properly initialized while y's ones are.
inline AutoDiffScalar<VectorXd> min(const AutoDiffScalar<VectorXd>& x,
                                    const AutoDiffScalar<VectorXd>& y) {
  internal::make_coherent(x.derivatives(), y.derivatives());
  return (x.value() < y.value() ? x : y);
}

// We have this specialization here because the Eigen-3.3.3's implementation of
// max for AutoDiffScalar does not call `make_coherent` function.
//
// For example, `max(x, y) + x` gives a runtime error if x.value() > y.value()
// but x's derivatives are not properly initialized while y's ones are.
// NOLINTNEXTLINE(build/include_what_you_use): Suppress false alarm.
inline AutoDiffScalar<VectorXd> max(const AutoDiffScalar<VectorXd>& x,
                                    const AutoDiffScalar<VectorXd>& y) {
  internal::make_coherent(x.derivatives(), y.derivatives());
  return (x.value() >= y.value() ? x : y);
}

inline const AutoDiffScalar<VectorXd> pow(const AutoDiffScalar<VectorXd>& a,
                                          double b) {
  using std::pow;
  return MakeAutoDiffScalar(pow(a.value(), b),
                            a.derivatives() * (b * pow(a.value(), b - 1)));
}

#endif

}  // namespace Eigen
