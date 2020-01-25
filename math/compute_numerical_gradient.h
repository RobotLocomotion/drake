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
 * value in y. `calc_fun` is responsible for properly resizing the output `y`
 * when it consists of an Eigen vector of Eigen::Dynamic size.
 *
 * @param x The point at which the numerical gradient is computed.
 * @param option The options for computing numerical gradient.
 * @tparam DerivedX an Eigen column vector.
 * @tparam DerivedY an Eigen column vector.
 * @tparam DerivedCalcX The type of x in the calc_fun. Must be an Eigen column
 * vector. It is possible to have DerivedCalcX being different from
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
Eigen::Matrix<typename DerivedX::Scalar, DerivedY::RowsAtCompileTime,
              DerivedX::RowsAtCompileTime>
ComputeNumericalGradient(
    std::function<void(const DerivedCalcX&, DerivedY* y)> calc_fun,
    const DerivedX& x,
    const NumericalGradientOption& option = NumericalGradientOption{
        NumericalGradientMethod::kForward}) {
  // All input Eigen types must be vectors templated on the same scalar type.
  typedef typename DerivedX::Scalar T;
  static_assert(
      is_eigen_vector_of<DerivedY, T>::value,
      "DerivedY must be templated on the same scalar type as DerivedX.");
  static_assert(
      is_eigen_vector_of<DerivedCalcX, T>::value,
      "DerivedCalcX must be templated on the same scalar type as DerivedX.");

  using std::abs;
  using std::max;

  // We need to evaluate f(x), only for the forward and backward schemes.
  Eigen::Matrix<T, DerivedY::RowsAtCompileTime, 1> y;
  if (option.method() != NumericalGradientMethod::kCentral)
    calc_fun(x, &y);

  // Now evaluate f(x + Δx) and f(x - Δx) along each dimension.
  Eigen::Matrix<T, DerivedX::RowsAtCompileTime, 1> x_plus = x;
  Eigen::Matrix<T, DerivedX::RowsAtCompileTime, 1> x_minus = x;
  Eigen::Matrix<T, DerivedY::RowsAtCompileTime, 1> y_plus, y_minus;
  Eigen::Matrix<T, DerivedY::RowsAtCompileTime, DerivedX::RowsAtCompileTime> J;
  for (int i = 0; i < x.rows(); ++i) {
    if (option.method() == NumericalGradientMethod::kForward ||
        option.method() == NumericalGradientMethod::kCentral) {
      x_plus(i) += max(option.perturbation_size(),
                       abs(x(i)) * option.perturbation_size());
      calc_fun(x_plus, &y_plus);
    }
    if (option.method() == NumericalGradientMethod::kBackward ||
        option.method() == NumericalGradientMethod::kCentral) {
      x_minus(i) -= max(option.perturbation_size(),
                        abs(x(i)) * option.perturbation_size());
      calc_fun(x_minus, &y_minus);
    }
    switch (option.method()) {
      case NumericalGradientMethod::kForward: {
        // Resize if we haven't yet.
        if (J.size() == 0) J.resize(y.rows(), x.rows());
        J.col(i) = (y_plus - y) / (x_plus(i) - x(i));
        break;
      }
      case NumericalGradientMethod::kBackward: {
        // Resize if we haven't yet.
        if (J.size() == 0) J.resize(y.rows(), x.rows());
        J.col(i) = (y - y_minus) / (x(i) - x_minus(i));
        break;
      }
      case NumericalGradientMethod::kCentral: {
        // Resize if we haven't yet.
        if (J.size() == 0) J.resize(y_plus.rows(), x.rows());
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
