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

#if EIGEN_VERSION_AT_LEAST(3, 2, 93)  // True when built via Drake superbuild.
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

  using std::pow;
  using std::log;
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

#endif  // EIGEN_VERSION...

#if !EIGEN_VERSION_AT_LEAST(3, 2, 93)  // False when built via Drake superbuild.
/// Overloads max to mimic std::max from <algorithm>.
/// Required for old, broken Eigen versions.
template <typename DerType>
const Eigen::AutoDiffScalar<DerType>& max(
    const Eigen::AutoDiffScalar<DerType>& x,
    const Eigen::AutoDiffScalar<DerType>& y) {
  return (x > y) ? x : y;
}
#endif  // EIGEN_VERSION...

}  // namespace Eigen

namespace drake {

/// Returns the autodiff scalar's value() as a double.  Never throws.
/// Overloads ExtractDoubleOrThrow from common/extract_double.h.
template <typename DerType>
double ExtractDoubleOrThrow(const Eigen::AutoDiffScalar<DerType>& scalar) {
  return static_cast<double>(scalar.value());
}

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
