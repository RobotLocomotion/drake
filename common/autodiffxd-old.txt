#pragma once

// This file is a modification of Eigen-3.3.3's AutoDiffScalar.h file which is
// available at
// https://gitlab.com/libeigen/eigen/-/blob/3.3.3/unsupported/Eigen/src/AutoDiff/AutoDiffScalar.h
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
#include <limits>
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
// Eigen's implementation of internal::make_coherent makes use of const_cast in
// order to promote zero sized derivatives. This however interferes badly with
// our caching system and produces unexpected behaviors. See #10971 for details.
// Therefore our implementation stops using internal::make_coherent and treats
// scalars with zero sized derivatives as constants, as it should.
//
// We also provide overloading of math functions for AutoDiffScalar<VectorXd>
// which return AutoDiffScalar<VectorXd> instead of an expression tree.
//
// See https://github.com/RobotLocomotion/drake/issues/6944 for more
// information. See also drake/common/autodiff_overloads.h.
//
// TODO(soonho-tri): Next time when we upgrade Eigen, please check if we still
// need these specializations.
//
// @note move-aware arithmetic
// Prior implementations of arithmetic overloads required construction of new
// objects at each operation, which induced costly heap allocations. In modern
// C++, it is possible to instead exploit move semantics to avoid allocation in
// many cases. In particular, the compiler can implicitly use moves to satisfy
// pass-by-value parameters in cases where moves are possible (move construction
// and assignment are available), and the storage in question is not needed
// afterward. This allows definitions of operators that pass and return by
// value, and only allocate when needed, as determined by the compiler. For C++
// considerations, see Scott Meyers' _Effective Modern C++_ Item 41. See #13985
// for more discussion of Drake considerations.
//
// @note default initialization
// Value initialization is not part of the Eigen::AutoDiffScalar contract nor
// part of the contract for our AutoDiffXd specialization of that template
// class. Thus no code should be written assuming that default-constructed
// AutoDiffXd objects will be value initialized. However, leaving the value
// uninitialized triggered hard-to-eliminate maybe-uninitialized warnings (not
// necessarily spurious) from some versions of gcc (not seen with clang).
// After determining that there is a negligible effect on performance, we
// decided (see PRs #15699 and #15792) to initialize the value member to avoid
// those warnings. Initializing to NaN (as opposed to zero) ensures that no code
// will function with uninitialized AutoDiffXd objects, just as it should not
// with any other Eigen::AutoDiffScalar.
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

  // Default constructor leaves the value effectively uninitialized and the
  // derivatives vector zero length.
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

  // Move construction and assignment are trivial, but need to be explicitly
  // requested, since we have user-declared copy and assignment operators.
  AutoDiffScalar(AutoDiffScalar&&) = default;
  AutoDiffScalar& operator=(AutoDiffScalar&&) = default;

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

  // The arithmetic operators below exploit move-awareness to avoid heap
  // allocations. See note `move-aware arithmetic` above. Particular details
  // will be called out below, the first time they appear.

  // Using a friend operator instead of a method allows the ADS parameter to be
  // used as storage when move optimizations are possible.
  friend inline AutoDiffScalar operator+(AutoDiffScalar a, const Scalar& b) {
    a += b;
    return a;
  }

  friend inline AutoDiffScalar operator+(const Scalar& a, AutoDiffScalar b) {
    b += a;
    return b;
  }

  // Compound assignment operators contain the primitive implementations, since
  // the choice of writable storage is clear. Binary operations invoke the
  // compound assignments.
  inline AutoDiffScalar& operator+=(const Scalar& other) {
    value() += other;
    return *this;
  }

  // It is possible that further overloads could exploit more move-awareness
  // here. However, overload ambiguities are difficult to resolve. Currently
  // only the left-hand operand is available for optimizations.  See #13985,
  // #14039 for discussion.
  template <typename OtherDerType>
  friend inline AutoDiffScalar<DerType> operator+(
      AutoDiffScalar<DerType> a, const AutoDiffScalar<OtherDerType>& b) {
    a += b;
    return a;
  }

  template <typename OtherDerType>
  inline AutoDiffScalar& operator+=(const AutoDiffScalar<OtherDerType>& other) {
    const bool has_this_der = m_derivatives.size() > 0;
    const bool has_both_der = has_this_der && (other.derivatives().size() > 0);
    m_value += other.value();
    if (has_both_der) {
      m_derivatives += other.derivatives();
    } else if (has_this_der) {
      // noop
    } else {
      m_derivatives = other.derivatives();
    }
    return *this;
  }

  friend inline AutoDiffScalar operator-(AutoDiffScalar a, const Scalar& b) {
    a -= b;
    return a;
  }

  // Scalar-on-the-left non-commutative operations must also contain primitive
  // implementations.
  friend inline AutoDiffScalar operator-(const Scalar& a, AutoDiffScalar b) {
    b.value() = a - b.value();
    b.derivatives() *= -1;
    return b;
  }

  inline AutoDiffScalar& operator-=(const Scalar& other) {
    m_value -= other;
    return *this;
  }

  template <typename OtherDerType>
  friend inline AutoDiffScalar<DerType> operator-(
      AutoDiffScalar<DerType> a, const AutoDiffScalar<OtherDerType>& other) {
    a -= other;
    return a;
  }

  template <typename OtherDerType>
  inline AutoDiffScalar& operator-=(const AutoDiffScalar<OtherDerType>& other) {
    const bool has_this_der = m_derivatives.size() > 0;
    const bool has_both_der = has_this_der && (other.derivatives().size() > 0);
    m_value -= other.value();
    if (has_both_der) {
      m_derivatives -= other.derivatives();
    } else if (has_this_der) {
      // noop
    } else {
      m_derivatives = -other.derivatives();
    }
    return *this;
  }

  // Phrasing unary negation as a value-passing friend permits some move
  // optimizations.
  friend inline AutoDiffScalar operator-(AutoDiffScalar a) {
    a.value() *= -1;
    a.derivatives() *= -1;
    return a;
  }

  friend inline AutoDiffScalar operator*(AutoDiffScalar a, const Scalar& b) {
    a *= b;
    return a;
  }

  friend inline AutoDiffScalar operator*(const Scalar& a, AutoDiffScalar b) {
    b *= a;
    return b;
  }

  friend inline AutoDiffScalar operator/(AutoDiffScalar a, const Scalar& b) {
    a /= b;
    return a;
  }

  friend inline AutoDiffScalar operator/(const Scalar& a, AutoDiffScalar b) {
    b.derivatives() *= Scalar(-a) / (b.value() * b.value());
    b.value() = a / b.value();
    return b;
  }

  template <typename OtherDerType>
  friend inline AutoDiffScalar<DerType> operator/(
      AutoDiffScalar<DerType> a, const AutoDiffScalar<OtherDerType>& b) {
    a /= b;
    return a;
  }

  template <typename OtherDerType>
  friend inline AutoDiffScalar<DerType> operator*(
      AutoDiffScalar<DerType> a, const AutoDiffScalar<OtherDerType>& b) {
    a *= b;
    return a;
  }

  inline AutoDiffScalar& operator*=(const Scalar& other) {
    m_value *= other;
    m_derivatives *= other;
    return *this;
  }

  template <typename OtherDerType>
  inline AutoDiffScalar& operator*=(const AutoDiffScalar<OtherDerType>& other) {
    const bool has_this_der = m_derivatives.size() > 0;
    const bool has_both_der = has_this_der && (other.derivatives().size() > 0);
    // Some of the math below may look tempting to rewrite using `*=`, but
    // performance measurement and analysis show that this formulation is
    // faster because it results in better expression tree optimization and
    // inlining.
    if (has_both_der) {
      m_derivatives = m_derivatives * other.value() +
                      other.derivatives() * m_value;
    } else if (has_this_der) {
      m_derivatives = m_derivatives * other.value();
    } else {
      m_derivatives = other.derivatives() * m_value;
    }
    m_value *= other.value();
    return *this;
  }

  inline AutoDiffScalar& operator/=(const Scalar& other) {
    m_value /= other;
    m_derivatives *= Scalar(1) / other;
    return *this;
  }

  template <typename OtherDerType>
  inline AutoDiffScalar& operator/=(const AutoDiffScalar<OtherDerType>& other) {
    auto& this_der = m_derivatives;
    const auto& other_der = other.derivatives();
    const bool has_this_der = m_derivatives.size() > 0;
    const bool has_both_der = has_this_der && (other.derivatives().size() > 0);
    const Scalar scale = Scalar(1) / (other.value() * other.value());
    if (has_both_der) {
      this_der *= other.value();
      this_der -= other_der * m_value;
      this_der *= scale;
    } else if (has_this_der) {
      this_der *= Scalar(1) / other.value();
    } else {
      this_der = other_der * -m_value * scale;
    }
    m_value /= other.value();
    return *this;
  }

 protected:
  // See class documentation above for why we are initializing the value here
  // even though that is not part of the Eigen::AutoDiffScalar contract.
  // Scalar is always double in this specialization.
  Scalar m_value{std::numeric_limits<double>::quiet_NaN()};
  DerType m_derivatives;
};

#define DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(FUNC, CODE) \
  inline AutoDiffScalar<VectorXd> FUNC(                         \
      AutoDiffScalar<VectorXd> x) {                             \
    EIGEN_UNUSED typedef double Scalar;                         \
    CODE;                                                       \
    return x;                                                   \
  }

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    abs, using std::abs;
    x.derivatives() *= (x.value() < 0 ? -1 : 1);
    x.value() = abs(x.value());)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    abs2, using numext::abs2;
    x.derivatives() *= (Scalar(2) * x.value());
    x.value() = abs2(x.value());)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    sqrt, using std::sqrt;
    Scalar sqrtx = sqrt(x.value());
    x.value() = sqrtx;
    x.derivatives() *= (Scalar(0.5) / sqrtx);)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    cos, using std::cos; using std::sin;
    x.derivatives() *= -sin(x.value());
    x.value() = cos(x.value());)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    sin, using std::sin; using std::cos;
    x.derivatives() *= cos(x.value());
    x.value() = sin(x.value());)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    exp, using std::exp;
    x.value() = exp(x.value());
    x.derivatives() *= x.value();)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    log, using std::log;
    x.derivatives() *= Scalar(1) / x.value();
    x.value() = log(x.value());)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    tan, using std::tan; using std::cos;
    auto sq = [](Scalar n) { return n * n; };
    x.derivatives() *= Scalar(1) / sq(cos(x.value()));
    x.value() = tan(x.value());)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    asin, using std::sqrt; using std::asin;
    auto sq = [](Scalar n) { return n * n; };
    x.derivatives() *= Scalar(1) / sqrt(1 - sq(x.value()));
    x.value() = asin(x.value());)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    acos, using std::sqrt; using std::acos;
    auto sq = [](Scalar n) { return n * n; };
    x.derivatives() *= Scalar(-1) / sqrt(1 - sq(x.value()));
    x.value() = acos(x.value());)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    atan, using std::atan;
    auto sq = [](Scalar n) { return n * n; };
    x.derivatives() *= Scalar(1) / (1 + sq(x.value()));
    x.value() = atan(x.value());)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    tanh, using std::cosh; using std::tanh;
    auto sq = [](Scalar n) { return n * n; };
    x.derivatives() *= Scalar(1) / sq(cosh(x.value()));
    x.value() = tanh(x.value());)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    sinh, using std::sinh; using std::cosh;
    x.derivatives() *= cosh(x.value());
    x.value() = sinh(x.value());)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    cosh, using std::sinh; using std::cosh;
    x.derivatives() *= sinh(x.value());
    x.value() = cosh(x.value());)

#undef DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY

// We have this specialization here because the Eigen-3.3.3's atan2
// implementation for AutoDiffScalar does not make a return with properly sized
// derivatives.
inline AutoDiffScalar<VectorXd> atan2(AutoDiffScalar<VectorXd> a,
                                      const AutoDiffScalar<VectorXd>& b) {
  const bool has_a_der = a.derivatives().size() > 0;
  const bool has_both_der = has_a_der && (b.derivatives().size() > 0);
  const double squared_hypot = a.value() * a.value() + b.value() * b.value();
  if (has_both_der) {
    a.derivatives() *= b.value();
    a.derivatives() -= a.value() * b.derivatives();
  } else if (has_a_der) {
    a.derivatives() *= b.value();
  } else {
    a.derivatives() = -a.value() * b.derivatives();
  }
  a.derivatives() /= squared_hypot;
  a.value() = std::atan2(a.value(), b.value());
  return a;
}

// Right-hand pass-by-value optimizations for atan2() are blocked by code in
// Eigen; see #14039.

inline AutoDiffScalar<VectorXd> pow(AutoDiffScalar<VectorXd> a, double b) {
  // TODO(rpoyner-tri): implementation seems fishy --see #14052.
  using std::pow;
  a.derivatives() *= b * pow(a.value(), b - 1);
  a.value() = pow(a.value(), b);
  return a;
}

// We have these implementations here because Eigen's implementations do not
// have consistent behavior when a == b. We enforce the following rules for that
// case:
// 1) If both a and b are ADS with non-empty derivatives, return a.
// 2) If both a and b are doubles, return a.
// 3) If one of a, b is a double, and the other is an ADS, return the ADS.
// 4) Treat ADS with empty derivatives as though they were doubles.
// Points (1) and (4) are handled here. Points (2) and (3) are already handled
// by Eigen's overloads.
// See https://gitlab.com/libeigen/eigen/-/issues/1870.
inline const AutoDiffScalar<VectorXd> min(const AutoDiffScalar<VectorXd>& a,
                                          const AutoDiffScalar<VectorXd>& b) {
  // If both a and b have derivatives, then their derivative sizes must match.
  DRAKE_ASSERT(
      a.derivatives().size() == 0 || b.derivatives().size() == 0 ||
      a.derivatives().size() == b.derivatives().size());
  // The smaller of a or b wins; ties go to a iff it has any derivatives.
  return ((a < b) || ((a == b) && (a.derivatives().size() != 0))) ? a : b;
}

// NOLINTNEXTLINE(build/include_what_you_use)
inline const AutoDiffScalar<VectorXd> max(const AutoDiffScalar<VectorXd>& a,
                                          const AutoDiffScalar<VectorXd>& b) {
  // If both a and b have derivatives, then their derivative sizes must match.
  DRAKE_ASSERT(
      a.derivatives().size() == 0 || b.derivatives().size() == 0 ||
      a.derivatives().size() == b.derivatives().size());
  // The larger of a or b wins; ties go to a iff it has any derivatives.
  return ((a > b) || ((a == b) && (a.derivatives().size() != 0))) ? a : b;
}

#endif

}  // namespace Eigen
