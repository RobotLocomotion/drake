#pragma once

#include <algorithm>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

namespace drake {
namespace systems {
namespace analysis {
namespace test {

/// Approximation technique for vector functions ℝ → ℝⁿ based on
/// a cubic Hermite interpolator. Returns a PiecewisePolynomial
/// instance to be queried through PiecewisePolynomial::value().
///
/// @note See InitialValueProblem::ApproximationTechnique and
///       InitialValueProblem::Approximate() definitions for
///       further reference.
template <typename T>
trajectories::PiecewisePolynomial<T> CubicVectorApproximationTechnique(
    const std::vector<T>& t_sequence,
    const std::vector<VectorX<T>>& x_sequence,
    const std::vector<VectorX<T>>& dxdt_sequence) {
  // Transforms values and first derivatives of the function to be
  // approximated into N-by-1 matrices (i.e. column vectors) to match
  // PiecewisePolynomial API.
  auto vector_to_matrix = [](const VectorX<T>& v) {
    return (MatrixX<T>(v.size(), 1) << v).finished();
  };
  std::vector<MatrixX<T>> x_matrix_sequence(x_sequence.size());
  std::transform(x_sequence.begin(), x_sequence.end(),
                 x_matrix_sequence.begin(), vector_to_matrix);
  std::vector<MatrixX<T>> dxdt_matrix_sequence(dxdt_sequence.size());
  std::transform(dxdt_sequence.begin(), dxdt_sequence.end(),
                 dxdt_matrix_sequence.begin(), vector_to_matrix);
  // Computes and returns the cubic spline.
  return trajectories::PiecewisePolynomial<T>::Cubic(
      t_sequence, x_matrix_sequence, dxdt_matrix_sequence);
}

/// Approximation technique for scalar functions ℝ → ℝ based on
/// a cubic Hermite interpolator. Returns a PiecewisePolynomial
/// instance to be queried through PiecewisePolynomial::scalarValue().
///
/// @note See ScalarInitialValueProblem::ApproximationTechnique and
///       ScalarInitialValueProblem::Approximate() definitions for
///       further reference.
template <typename T>
trajectories::PiecewisePolynomial<T> CubicApproximationTechnique(
    const std::vector<T>& t_sequence,
    const std::vector<T>& x_sequence,
    const std::vector<T>& dxdt_sequence) {
  // Transforms values and first derivatives of the function to be
  // approximated into one element matrices to match PiecewisePolynomial
  // API.
  auto scalar_to_matrix = [](const T& v) {
    return (MatrixX<T>(1, 1) << v).finished();
  };
  std::vector<MatrixX<T>> x_matrix_sequence(x_sequence.size());
  std::transform(x_sequence.begin(), x_sequence.end(),
                 x_matrix_sequence.begin(), scalar_to_matrix);
  std::vector<MatrixX<T>> dxdt_matrix_sequence(dxdt_sequence.size());
  std::transform(dxdt_sequence.begin(), dxdt_sequence.end(),
                 dxdt_matrix_sequence.begin(), scalar_to_matrix);
  // Computes and returns the cubic spline.
  return trajectories::PiecewisePolynomial<T>::Cubic(
      t_sequence, x_matrix_sequence, dxdt_matrix_sequence);
}

}  // namespace test
}  // namespace analysis
}  // namespace systems
}  // namespace drake
