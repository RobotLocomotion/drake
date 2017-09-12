/// @file
/// Overloads for STL mathematical operations on AutoDiffScalar.
///
/// Used via argument-dependent lookup (ADL). These functions appear
/// in the Eigen namespace so that ADL can automatically choose between
/// the STL version and the overloaded version to match the type of the
/// arguments. The proper use would be e.g.
///
/// \code{.cc}
///    void mymethod() {
///       using std::isinf;
///       isinf(myval);
///    }
/// \endcode{}
///
/// @note The if_then_else and cond functions for AutoDiffScalar are in
/// namespace drake because cond is defined in namespace drake in
/// "drake/common/cond.h" file.

#pragma once

#include <cmath>
#include <limits>

#include <Eigen/Dense>
#include <unsupported/Eigen/AutoDiff>

#include "drake/common/cond.h"
#include "drake/common/drake_assert.h"
#include "drake/common/dummy_value.h"

namespace Eigen {

/// Overloads round to mimic std::round from <cmath>.
template <typename DerType>
double round(const Eigen::AutoDiffScalar<DerType>& x) {
  using std::round;
  return round(x.value());
}

/// Overloads isinf to mimic std::isinf from <cmath>.
template <typename DerType>
bool isinf(const Eigen::AutoDiffScalar<DerType>& x) {
  using std::isinf;
  return isinf(x.value());
}

/// Overloads isnan to mimic std::isnan from <cmath>.
template <typename DerType>
bool isnan(const Eigen::AutoDiffScalar<DerType>& x) {
  using std::isnan;
  return isnan(x.value());
}

/// Overloads floor to mimic std::floor from <cmath>.
template <typename DerType>
double floor(const Eigen::AutoDiffScalar<DerType>& x) {
  using std::floor;
  return floor(x.value());
}

/// Overloads ceil to mimic std::ceil from <cmath>.
template <typename DerType>
double ceil(const Eigen::AutoDiffScalar<DerType>& x) {
  using std::ceil;
  return ceil(x.value());
}

/// Overloads copysign from <cmath>.
template <typename DerType, typename T>
Eigen::AutoDiffScalar<DerType> copysign(const Eigen::AutoDiffScalar<DerType>& x,
                                        const T& y) {
  using std::isnan;
  if (isnan(x)) return (y >= 0) ? NAN : -NAN;
  if ((x < 0 && y >= 0) || (x >= 0 && y < 0))
    return -x;
  else
    return x;
}

/// Overloads copysign from <cmath>.
template <typename DerType>
double copysign(double x, const Eigen::AutoDiffScalar<DerType>& y) {
  using std::isnan;
  if (isnan(x)) return (y >= 0) ? NAN : -NAN;
  if ((x < 0 && y >= 0) || (x >= 0 && y < 0))
    return -x;
  else
    return x;
}

/// Overloads pow for an AutoDiffScalar base and exponent, implementing the
/// chain rule.
template <typename DerTypeA, typename DerTypeB>
Eigen::AutoDiffScalar<typename DerTypeA::PlainObject> pow(
    const Eigen::AutoDiffScalar<DerTypeA>& base,
    const Eigen::AutoDiffScalar<DerTypeB>& exponent) {
  // The two AutoDiffScalars being exponentiated must have the same matrix
  // type. This includes, but is not limited to, the same scalar type and
  // the same dimension.
  static_assert(std::is_same<typename DerTypeA::PlainObject,
                             typename DerTypeB::PlainObject>::value,
                "The derivative types must match.");

  const auto& x = base.value();
  const auto& xgrad = base.derivatives();
  const auto& y = exponent.value();
  const auto& ygrad = exponent.derivatives();

  using std::log;
  using std::pow;
  const auto x_to_the_y = pow(x, y);
  return Eigen::MakeAutoDiffScalar(
      // The value is x ^ y.
      x_to_the_y,
      // The multivariable chain rule states:
      // df/dv_i = (∂f/∂x * dx/dv_i) + (∂f/∂y * dy/dv_i)
      // ∂f/∂x is y*x^(y-1)
      y * pow(x, y - 1) * xgrad +
          // ∂f/∂y is (x^y)*ln(x)
          x_to_the_y * log(x) * ygrad);
}

#if !defined(DRAKE_DOXYGEN_CXX)
// Specializations for AutoDiffXd.
//
// AutoDiffScalar (ADS) tries to call internal::make_coherent to promote empty
// derivatives. However, it fails to do the promotion when an operand is an
// expression tree (i.e. CwiseBinaryOp). Our solution is to provide special
// overloading for AutoDiffXd, which evaluates terms immediately and returns an
// AutoDiffXd instead of expression trees. To do this, first, we provide a
// template specialization, AutoDiffScalar<VectorXd>, which modifies the return
// type of operators. Second, we provide overloading of math functions (such as
// sin, cos, pow, log, exp, ...) for AutoDiffXd.
//
// See https://github.com/RobotLocomotion/drake/issues/6944 for more
// information.

// The following class definition is a modification from the
// `unsupported/Eigen/src/AutoDiff/AutoDiffScalar.h` file in Eigen's source
// code.
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
                          OtherDerType>::type>::Scalar>::value &&
              internal::is_convertible<OtherDerType, DerType>::value,
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

  // Modified for Drake.
  inline const AutoDiffScalar<DerType> operator+(const Scalar& other) const {
    return AutoDiffScalar<DerType&>(m_value + other, m_derivatives);
  }

  // Modified for Drake.
  friend inline const AutoDiffScalar<DerType> operator+(
      const Scalar& a, const AutoDiffScalar& b) {
    return AutoDiffScalar<DerType>(a + b.value(), b.derivatives());
  }

  inline AutoDiffScalar& operator+=(const Scalar& other) {
    value() += other;
    return *this;
  }

  // Modified for Drake.
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

  // Modified for Drake.
  inline const AutoDiffScalar<DerType> operator-(const Scalar& b) const {
    return AutoDiffScalar<DerType>(m_value - b, m_derivatives);
  }

  // Modified for Drake.
  friend inline const AutoDiffScalar<DerType> operator-(
      const Scalar& a, const AutoDiffScalar& b) {
    return AutoDiffScalar<DerType>(a - b.value(), -b.derivatives());
  }

  inline AutoDiffScalar& operator-=(const Scalar& other) {
    value() -= other;
    return *this;
  }

  // Modified for Drake.
  template <typename OtherDerType>
  inline const AutoDiffScalar<DerType> operator-(
      const AutoDiffScalar<OtherDerType>& other) const {
    internal::make_coherent(m_derivatives, other.derivatives());
    return AutoDiffScalar<
        CwiseBinaryOp<internal::scalar_difference_op<Scalar>, const DerType,
                      const typename internal::remove_all<OtherDerType>::type>>(
        m_value - other.value(), m_derivatives - other.derivatives());
  }

  template <typename OtherDerType>
  inline AutoDiffScalar& operator-=(const AutoDiffScalar<OtherDerType>& other) {
    *this = *this - other;
    return *this;
  }

  // Modified for Drake.
  inline const AutoDiffScalar<DerType> operator-() const {
    return AutoDiffScalar<
        CwiseUnaryOp<internal::scalar_opposite_op<Scalar>, const DerType>>(
        -m_value, -m_derivatives);
  }

  // Modified for Drake.
  inline const AutoDiffScalar<DerType> operator*(const Scalar& other) const {
    return MakeAutoDiffScalar(m_value * other, m_derivatives * other);
  }

  // Modified for Drake.
  friend inline const AutoDiffScalar<DerType> operator*(
      const Scalar& other, const AutoDiffScalar& a) {
    return MakeAutoDiffScalar(a.value() * other, a.derivatives() * other);
  }

  // Modified for Drake.
  inline const AutoDiffScalar<DerType> operator/(const Scalar& other) const {
    return MakeAutoDiffScalar(m_value / other,
                              (m_derivatives * (Scalar(1) / other)));
  }

  // Modified for Drake.
  friend inline const AutoDiffScalar<DerType> operator/(
      const Scalar& other, const AutoDiffScalar& a) {
    return MakeAutoDiffScalar(
        other / a.value(),
        a.derivatives() * (Scalar(-other) / (a.value() * a.value())));
  }

  // Modified for Drake.
  template <typename OtherDerType>
  inline const AutoDiffScalar<DerType> operator/(
      const AutoDiffScalar<OtherDerType>& other) const {
    internal::make_coherent(m_derivatives, other.derivatives());
    return MakeAutoDiffScalar(
        m_value / other.value(),
        ((m_derivatives * other.value()) - (other.derivatives() * m_value)) *
            (Scalar(1) / (other.value() * other.value())));
  }

  // Modified for Drake.
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

#define DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_BINARY(FUNC, CODE)              \
  inline const AutoDiffScalar<VectorXd> FUNC(                                 \
      const AutoDiffScalar<VectorXd>& a, const AutoDiffScalar<VectorXd>& b) { \
    EIGEN_UNUSED typedef double Scalar;                                       \
    CODE;                                                                     \
  }

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_BINARY(
    pow, internal::make_coherent(a.derivatives(), b.derivatives());
    const auto& x = a.value(); const auto& xgrad = a.derivatives();
    const auto& y = b.value(); const auto& ygrad = b.derivatives();
    using std::log; using std::pow; const auto x_to_the_y = pow(x, y);
    return Eigen::MakeAutoDiffScalar(
        // The value is x ^ y.
        x_to_the_y,
        // The multivariable chain rule states:
        // df/dv_i = (∂f/∂x * dx/dv_i) + (∂f/∂y * dy/dv_i)
        // ∂f/∂x is y*x^(y-1)
        y* pow(x, y - 1) * xgrad +
            // ∂f/∂y is (x^y)*ln(x)
            x_to_the_y * log(x) * ygrad);)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_BINARY(
    atan2, internal::make_coherent(a.derivatives(), b.derivatives());
    using std::atan2; AutoDiffScalar<VectorXd> ret;
    ret.value() = atan2(a.value(), b.value());
    const double squared_hypot = a.value() * a.value() + b.value() * b.value();
    // if (squared_hypot==0) the derivation is undefined and the following
    // results in a NaN:
    ret.derivatives() = (a.derivatives() * b.value() -
                         a.value() * b.derivatives()) /
                        squared_hypot;
    return ret;)

#undef DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_BINARY

#define DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(FUNC, CODE) \
  inline const AutoDiffScalar<VectorXd> FUNC(                   \
      const AutoDiffScalar<VectorXd>& x) {                      \
    EIGEN_UNUSED typedef double Scalar;                         \
    CODE;                                                       \
  }

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    abs, using std::abs;
    return MakeAutoDiffScalar(abs(x.value()),
                              x.derivatives() * (x.value() < 0 ? -1 : 1));)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    abs2, using numext::abs2;
    return MakeAutoDiffScalar(abs2(x.value()),
                              x.derivatives() * (Scalar(2) * x.value()));)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    sqrt, using std::sqrt; Scalar sqrtx = sqrt(x.value());
    return MakeAutoDiffScalar(sqrtx, x.derivatives() * (Scalar(0.5) / sqrtx));)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    cos, using std::cos; using std::sin;
    return MakeAutoDiffScalar(cos(x.value()),
                              x.derivatives() * (-sin(x.value())));)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    sin, using std::sin; using std::cos;
    return MakeAutoDiffScalar(sin(x.value()),
                              x.derivatives() * cos(x.value()));)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    exp, using std::exp; Scalar expx = exp(x.value());
    return MakeAutoDiffScalar(expx, x.derivatives() * expx);)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    log, using std::log;
    return MakeAutoDiffScalar(log(x.value()),
                              x.derivatives() * (Scalar(1) / x.value()));)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    tan, using std::tan; using std::cos;
    return MakeAutoDiffScalar(tan(x.value()),
                              x.derivatives() *
                                  (Scalar(1) / numext::abs2(cos(x.value()))));)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    asin, using std::sqrt; using std::asin; return MakeAutoDiffScalar(
        asin(x.value()),
        x.derivatives() * (Scalar(1) / sqrt(1 - numext::abs2(x.value()))));)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    acos, using std::sqrt; using std::acos; return MakeAutoDiffScalar(
        acos(x.value()),
        x.derivatives() * (Scalar(-1) / sqrt(1 - numext::abs2(x.value()))));)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    tanh, using std::cosh; using std::tanh;
    return MakeAutoDiffScalar(tanh(x.value()),
                              x.derivatives() *
                                  (Scalar(1) / numext::abs2(cosh(x.value()))));)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    sinh, using std::sinh; using std::cosh;
    return MakeAutoDiffScalar(sinh(x.value()),
                              x.derivatives() * cosh(x.value()));)

DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY(
    cosh, using std::sinh; using std::cosh;
    return MakeAutoDiffScalar(cosh(x.value()),
                              x.derivatives() * sinh(x.value()));)

#undef DRAKE_EIGEN_AUTODIFFXD_DECLARE_GLOBAL_UNARY

inline const AutoDiffScalar<VectorXd> pow(const AutoDiffScalar<VectorXd>& a,
                                          double b) {
  using std::pow;
  return MakeAutoDiffScalar(pow(a.value(), b),
                            a.derivatives() * (b * pow(a.value(), b - 1)));
}

#endif

}  // namespace Eigen

namespace drake {

/// Returns the autodiff scalar's value() as a double.  Never throws.
/// Overloads ExtractDoubleOrThrow from common/extract_double.h.
template <typename DerType>
double ExtractDoubleOrThrow(const Eigen::AutoDiffScalar<DerType>& scalar) {
  return static_cast<double>(scalar.value());
}

/// Specializes common/dummy_value.h.
template <typename DerType>
struct dummy_value<Eigen::AutoDiffScalar<DerType>> {
  static constexpr Eigen::AutoDiffScalar<DerType> get() {
    constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
    DerType derivatives;
    derivatives.fill(kNaN);
    return Eigen::AutoDiffScalar<DerType>(kNaN, derivatives);
  }
};

/// Provides if-then-else expression for Eigen::AutoDiffScalar type. To support
/// Eigen's generic expressions, we use casting to the plain object after
/// applying Eigen::internal::remove_all. It is based on the Eigen's
/// implementation of min/max function for AutoDiffScalar type
/// (https://bitbucket.org/eigen/eigen/src/10a1de58614569c9250df88bdfc6402024687bc6/unsupported/Eigen/src/AutoDiff/AutoDiffScalar.h?at=default&fileviewer=file-view-default#AutoDiffScalar.h-546).
template <typename DerType1, typename DerType2>
inline Eigen::AutoDiffScalar<
    typename Eigen::internal::remove_all<DerType1>::type::PlainObject>
if_then_else(bool f_cond, const Eigen::AutoDiffScalar<DerType1>& x,
             const Eigen::AutoDiffScalar<DerType2>& y) {
  typedef Eigen::AutoDiffScalar<
      typename Eigen::internal::remove_all<DerType1>::type::PlainObject>
      ADS1;
  typedef Eigen::AutoDiffScalar<
      typename Eigen::internal::remove_all<DerType2>::type::PlainObject>
      ADS2;
  static_assert(std::is_same<ADS1, ADS2>::value,
                "The derivative types must match.");
  return f_cond ? ADS1(x) : ADS2(y);
}

/// Provides special case of cond expression for Eigen::AutoDiffScalar type.
template <typename DerType, typename... Rest>
Eigen::AutoDiffScalar<
    typename Eigen::internal::remove_all<DerType>::type::PlainObject>
cond(bool f_cond, const Eigen::AutoDiffScalar<DerType>& e_then, Rest... rest) {
  return if_then_else(f_cond, e_then, cond(rest...));
}

}  // namespace drake
