#pragma once

#include <Eigen/Core>

namespace drake {
namespace internal {

/* Computes the matrix exponential eᴹ. The matrix exponential of M is
defined by `∑ₖ₌₀᪲ (Mᵏ / k!)` and can be used to solve linear ordinary
differential equations: the solution of `y' = My` with the initial
condition `y(0) = y_0` is given by `y(t) = eᴹ y_0`. */
Eigen::MatrixXd CalcMatrixExponential(const Eigen::MatrixXd& M);

}  // namespace internal
}  // namespace drake
