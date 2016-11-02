/// @file
/// Overloads for STL mathematical operations on AutoDiffScalar.
///
/// Used via argument-dependent lookup (ADL). These functions appear
/// in the global namespace so that ADL can automatically choose between
/// the STL version and the overloaded version to match the type of the
/// arguments. We could alternatively have placed them in the Eigen
/// namespace, but since we don't own that, we don't wish to pollute it.

#pragma once

#include <cmath>

#include <Eigen/Dense>
#include <unsupported/Eigen/AutoDiff>

/// Overloads round to mimic std::round from <cmath>.
template <typename DerType>
double round(const Eigen::AutoDiffScalar<DerType>& x) {
  using std::round;
  return round(x.value());
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
