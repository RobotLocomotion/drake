#pragma once

#include <Eigen/Dense>

#include "drake/math/gradient.h"
#include "drake/math/gradient_util.h"

namespace drake {
namespace math {
/** Computes the normalized vector, optionally with its gradient and second
derivative.
@param[in]  x        An N x 1 vector to be normalized. Must not be zero.
@param[out] x_norm   The normalized vector (N x 1).
@param[out] dx_norm
    If non-null, returned as an N x N matrix,
    where dx_norm(i,j) = D x_norm(i)/D x(j).
@param[out] ddx_norm
    If non-null, and dx_norm is non-null, returned as an N^2 x N matrix,
    where ddx_norm.col(j) = D dx_norm/D x(j), with dx_norm stacked
    columnwise.

(D x / D y above means partial derivative of x with respect to y.) */
template <typename Derived>
void NormalizeVector(
    const Eigen::MatrixBase<Derived>& x,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    typename Derived::PlainObject& x_norm,
    typename drake::math::Gradient<Derived, Derived::RowsAtCompileTime,
                                   1>::type* dx_norm = nullptr,
    typename drake::math::Gradient<Derived, Derived::RowsAtCompileTime,
                                   2>::type* ddx_norm = nullptr) {
  typename Derived::Scalar xdotx = x.squaredNorm();
  typename Derived::Scalar norm_x = sqrt(xdotx);
  x_norm = x / norm_x;

  if (dx_norm) {
    dx_norm->setIdentity(x.rows(), x.rows());
    (*dx_norm) -= x * x.transpose() / xdotx;
    (*dx_norm) /= norm_x;

    if (ddx_norm) {
      auto dx_norm_transpose = transposeGrad(*dx_norm, x.rows());
      auto minus_ddx_norm_times_norm = matGradMultMat(
          x_norm, x_norm.transpose(), (*dx_norm), dx_norm_transpose);
      auto dnorm_inv = -x.transpose() / (xdotx * norm_x);
      (*ddx_norm) = -minus_ddx_norm_times_norm / norm_x;
      auto temp = (*dx_norm) * norm_x;
      typename Derived::Index n = x.rows();
      for (int col = 0; col < n; col++) {
        auto column_as_matrix = (dnorm_inv(0, col) * temp);
        for (int row_block = 0; row_block < n; row_block++) {
          ddx_norm->block(row_block * n, col, n, 1) +=
              column_as_matrix.col(row_block);
        }
      }
    }
  }
}
}  // namespace math
}  // namespace drake
