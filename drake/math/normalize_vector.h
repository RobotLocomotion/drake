#pragma once

#include <Eigen/Dense>
#include "drake/math/gradient.h"
#include "drake/util/drakeGradientUtil.h"

namespace drake {
namespace math {
/** Computes the normalized vector, together with its gradient and second derivatives
    @param x A \p N x 1 vector to be normalized
    @param x_norm The normalized vector
    @param dx_norm A @p N x @p N matrix, @p dx_norm(i,j) is the partial derivative
    of @p x_norm(i) w.r.t @p x(j)
    @param ddx_norm
 */
template <typename Derived>
void NormalizeVector(
    const Eigen::MatrixBase<Derived>& x, typename Derived::PlainObject& x_norm,
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
      auto ddx_norm_times_norm = -matGradMultMat(x_norm, x_norm.transpose(),
                                                 (*dx_norm), dx_norm_transpose);
      auto dnorm_inv = -x.transpose() / (xdotx * norm_x);
      (*ddx_norm) = ddx_norm_times_norm / norm_x;
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
} // namespace math
} // namespace drake