#include "drake/common/trajectories/path_parameterized_trajectory.h"

#include <fmt/ostream.h>

#include "drake/common/drake_assert.h"

namespace drake {
namespace trajectories {

template <typename T>
PathParameterizedTrajectory<T>::PathParameterizedTrajectory(
    const Trajectory<T>& path, const Trajectory<T>& parameterization)
    : path_{path.Clone()}, parameterization_{parameterization.Clone()} {
  DRAKE_DEMAND(parameterization.rows() == 1);
  DRAKE_DEMAND(parameterization.cols() == 1);
}

template <typename T>
std::unique_ptr<Trajectory<T>> PathParameterizedTrajectory<T>::Clone() const {
  return std::make_unique<PathParameterizedTrajectory<T>>(*path_,
                                                          *parameterization_);
}

template <typename T>
MatrixX<T> PathParameterizedTrajectory<T>::value(const T &time) const {
  return path_->value(parameterization_->value(time)(0, 0));
}

template <typename T>
T PathParameterizedTrajectory<T>::BellPolynomial(int n, int k,
                                                 const VectorX<T> x) const {
  if (n == 0 && k == 0) {
    return 1;
  } else if (n == 0 || k == 0) {
    return 0;
  }
  T polynomial = 0;
  T a = 1;
  for (int ii = 1; ii < n - k + 2; ii++) {
    polynomial += a * BellPolynomial(n - ii, k - 1, x) * x[ii - 1];
    a = a * (n - ii) / ii;
  }
  return polynomial;
}

template <typename T>
MatrixX<T> PathParameterizedTrajectory<T>::DoEvalDerivative(
    const T& t, int derivative_order) const {
  if (derivative_order == 0) {
    return value(t);
  } else if (derivative_order > 0) {
    VectorX<T> s_derivatives(derivative_order);
    for (int order = 0; order < derivative_order; order++) {
      s_derivatives(order) =
          parameterization_->EvalDerivative(t, order + 1)(0, 0);
    }
    // Derivative is calculated using Faà di Bruno's formula with Bell
    // polynomials: https://en.wikipedia.org/wiki/Fa%C3%A0_di_Bruno%27s_formula
    MatrixX<T> derivative = MatrixX<T>::Zero(rows(), cols());
    for (int order = 1; order <= derivative_order; order++) {
      MatrixX<T> path_partial =
          path_->EvalDerivative(parameterization_->value(t)(0, 0), order);
      derivative +=
          path_partial * BellPolynomial(derivative_order, order, s_derivatives);
    }
    return derivative;
  } else {
    throw std::invalid_argument(
        fmt::format("Invalid derivative order ({}). The derivative order must "
                    "be greater than or equal to 0.",
                    derivative_order));
  }
}

}  // namespace trajectories
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::trajectories::PathParameterizedTrajectory)
