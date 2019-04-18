#pragma once

#include <algorithm>

#include "drake/common/eigen_types.h"

namespace drake {
namespace math {

enum class NumericalGradientMethod {
  kForward,  ///< Compute the gradient as (f(x + Δx) - f(x)) / Δx, with Δx > 0
  kBackward,  ///< Compute the gradient as (f(x) - f(x - Δx)) / Δx, with Δx > 0
  kCentral,  ///< Compute the gradient as (f(x + Δx) - f(x - Δx)) / (2Δx), with
             ///< Δx > 0
};

struct NumericalGradientOption {
  NumericalGradientMethod method{NumericalGradientMethod::kCentral};
  // The step length Δx is max(|x(i)| * delta_x, delta_x) in each dimension.
  double delta_x{1E-7};
};

/**
 * Compute the gradient of a function f(x) through numerical difference.
 * @param calc_fun calc_fun(x, &y) computes the value of f(x), and stores the
 * value in y.
 * @param x The point at which the numerical gradient is computed.
 * @param option The options for computing numerical gradient.
 * @tparam DerivedX an Eigen column vector of double.
 * @tparam DerivedY an Eigen column vector of double.
 * @tparam DerivedCalcX The type of x in the calc_fun. Must be an Eigen column
 * vector of double.
 * @return gradient a matrix of size x.rows() x y.rows(). gradient(i, j) is
 * ∂f(i) / ∂x(j)
 */
template <typename DerivedX, typename DerivedY, typename DerivedCalcX>
typename std::enable_if<is_eigen_vector_of<DerivedX, double>::value &&
                            is_eigen_vector_of<DerivedY, double>::value &&
                            is_eigen_vector_of<DerivedCalcX, double>::value,
                        Eigen::Matrix<double, DerivedY::RowsAtCompileTime,
                                      DerivedX::RowsAtCompileTime>>::type
ComputeNumericalGradient(
    std::function<void(const DerivedCalcX&, DerivedY* y)> calc_fun,
    const DerivedX& x, const NumericalGradientOption& option = {}) {
  // First evaluates f(x)
  Eigen::Matrix<double, DerivedY::RowsAtCompileTime, 1> y;
  calc_fun(x, &y);

  // Now evaluate f(x + Δx) and f(x - Δx) along each dimension.
  Eigen::Matrix<double, DerivedX::RowsAtCompileTime, 1> x_plus(x.rows());
  Eigen::Matrix<double, DerivedX::RowsAtCompileTime, 1> x_minus(x.rows());
  Eigen::Matrix<double, DerivedY::RowsAtCompileTime, 1> y_plus, y_minus;
  Eigen::Matrix<double, DerivedY::RowsAtCompileTime,
                DerivedX::RowsAtCompileTime>
      J(y.rows(), x.rows());
  for (int i = 0; i < x.rows(); ++i) {
    if (option.method == NumericalGradientMethod::kForward ||
        option.method == NumericalGradientMethod::kCentral) {
      x_plus = x;
      x_plus(i) += std::max(option.delta_x, std::abs(x(i)) * option.delta_x);
      calc_fun(x_plus, &y_plus);
    }
    if (option.method == NumericalGradientMethod::kBackward ||
        option.method == NumericalGradientMethod::kCentral) {
      x_minus = x;
      x_minus(i) -= std::max(option.delta_x, std::abs(x(i)) * option.delta_x);
      calc_fun(x_minus, &y_minus);
    }
    switch (option.method) {
      case NumericalGradientMethod::kForward: {
        J.col(i) = (y_plus - y) / (x_plus(i) - x(i));
        break;
      }
      case NumericalGradientMethod::kBackward: {
        J.col(i) = (y - y_minus) / (x(i) - x_minus(i));
        break;
      }
      case NumericalGradientMethod::kCentral: {
        J.col(i) = (y_plus - y_minus) / (x_plus(i) - x_minus(i));
        break;
      }
    }
  }
  return J;
}
}  // namespace math
}  // namespace drake
