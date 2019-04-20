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

class NumericalGradientOption {
 public:
  /**
   * @param function_accuracy The accuracy of evaluating function f(x). For
   * double-valued functions (with magnitude around 1), the accuracy is usually
   * about 1E-15.
   */
  explicit NumericalGradientOption(NumericalGradientMethod method,
                                   double function_accuracy = 1e-15)
      : method_{method} {
    perturbation_size_ = method == NumericalGradientMethod::kCentral
                             ? std::cbrt(function_accuracy)
                             : std::sqrt(function_accuracy);
  }

  NumericalGradientMethod method() const { return method_; }

  double perturbation_size() const { return perturbation_size_; }

 private:
  NumericalGradientMethod method_{NumericalGradientMethod::kCentral};

  /** The step length Δx is max(|x(i)| * perturbation_size, perturbation_size)
   * in each dimension. If function f is evaluated with accuracy 10⁻¹⁵, then
   * a first-order method (forward or backward difference) should use a
   * perturbation of √10⁻¹⁵ ≈ 10⁻⁷, a second-order method (central
   * difference) should use ∛10⁻¹⁵≈ 10⁻⁵. The interested reader could refer
   * to section 8.6 of Practical Optimization by Philip E. Gill, Walter Murray
   * and Margaret H. Wright.
   */
  double perturbation_size_{NAN};
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
 * vector of double. It is possible to have DerivedCalcX being different from
 * DerivedX, for example, `calc_fun` could be solvers::EvaluatorBase(const
 * Eigen::Ref<const Eigen::VectorXd>&, Eigen::VectorXd*), but `x` could be of
 * type Eigen::VectorXd.
 * TODO(hongkai.dai): understand why the default template DerivedCalcX =
 * DerivedX doesn't compile when I instantiate
 * ComputeNumericalGradient<DerivedX, DerivedY>(calc_fun, x);
 * @retval gradient a matrix of size x.rows() x y.rows(). gradient(i, j) is
 * ∂f(i) / ∂x(j)
 *
 * Examples:
 * @code{cc}
 * // Create a std::function from a lambda expression.
 * std::function<void (const Eigen::Vector2d&, Vector3d*)> foo = [](const
 * Eigen::Vector2d& x, Vector3d*y) { (*y)(0) = x(0); (*y)(1) = x(0) * x(1);
 * (*y)(2) = x(0) * std::sin(x(1));};
 * Eigen::Vector3d x_eval(1, 2, 3);
 * auto J = ComputeNumericalGradient(foo, x_eval);
 * // Note that if we pass in a lambda to ComputeNumericalGradient, then
 * // ComputeNumericalGradient has to instantiate the template types explicitly,
 * // as in this example. The issue of template deduction with std::function is
 * // explained in
 * //
 * https://stackoverflow.com/questions/48529410/template-arguments-deduction-failed-passing-func-pointer-to-stdfunction
 * auto bar = [](const Eigen::Vector2d& x, Eigen::Vector2d* y) {*y = x; };
 * auto J2 = ComputeNumericalGradient<Eigen::Vector2d,
 * Eigen::Vector2d, Eigen::Vector2d>(bar, Eigen::Vector2d(2, 3));
 *
 * @endcode
 */
template <typename DerivedX, typename DerivedY, typename DerivedCalcX>
typename std::enable_if<is_eigen_vector_of<DerivedX, double>::value &&
                            is_eigen_vector_of<DerivedY, double>::value &&
                            is_eigen_vector_of<DerivedCalcX, double>::value,
                        Eigen::Matrix<double, DerivedY::RowsAtCompileTime,
                                      DerivedX::RowsAtCompileTime>>::type
ComputeNumericalGradient(
    std::function<void(const DerivedCalcX&, DerivedY* y)> calc_fun,
    const DerivedX& x,
    const NumericalGradientOption& option = NumericalGradientOption{
        NumericalGradientMethod::kForward}) {
  // First evaluate f(x).
  Eigen::Matrix<double, DerivedY::RowsAtCompileTime, 1> y;
  calc_fun(x, &y);

  // Now evaluate f(x + Δx) and f(x - Δx) along each dimension.
  Eigen::Matrix<double, DerivedX::RowsAtCompileTime, 1> x_plus = x;
  Eigen::Matrix<double, DerivedX::RowsAtCompileTime, 1> x_minus = x;
  Eigen::Matrix<double, DerivedY::RowsAtCompileTime, 1> y_plus, y_minus;
  Eigen::Matrix<double, DerivedY::RowsAtCompileTime,
                DerivedX::RowsAtCompileTime>
      J(y.rows(), x.rows());
  for (int i = 0; i < x.rows(); ++i) {
    if (option.method() == NumericalGradientMethod::kForward ||
        option.method() == NumericalGradientMethod::kCentral) {
      x_plus(i) += std::max(option.perturbation_size(),
                            std::abs(x(i)) * option.perturbation_size());
      calc_fun(x_plus, &y_plus);
    }
    if (option.method() == NumericalGradientMethod::kBackward ||
        option.method() == NumericalGradientMethod::kCentral) {
      x_minus(i) -= std::max(option.perturbation_size(),
                             std::abs(x(i)) * option.perturbation_size());
      calc_fun(x_minus, &y_minus);
    }
    switch (option.method()) {
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
    // Restore perturbed values.
    x_plus(i) = x_minus(i) = x(i);
  }
  return J;
}
}  // namespace math
}  // namespace drake
