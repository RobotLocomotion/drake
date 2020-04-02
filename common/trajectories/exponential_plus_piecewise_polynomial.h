#pragma once

#include <memory>
#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

namespace drake {
namespace trajectories {

/**
 * y(t) = K * exp(A * (t - t_j)) * alpha.col(j) + piecewise_polynomial_part(t)
 *
 * @tparam_double_only
 */
template <typename T>
class ExponentialPlusPiecewisePolynomial final
    : public PiecewiseTrajectory<T> {
 public:
  // We are final, so this is okay.
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ExponentialPlusPiecewisePolynomial)

  ExponentialPlusPiecewisePolynomial() = default;

  template <typename DerivedK, typename DerivedA, typename DerivedAlpha>
  ExponentialPlusPiecewisePolynomial(
      const Eigen::MatrixBase<DerivedK>& K,
      const Eigen::MatrixBase<DerivedA>& A,
      const Eigen::MatrixBase<DerivedAlpha>& alpha,
      const PiecewisePolynomial<T>& piecewise_polynomial_part)
      : PiecewiseTrajectory<T>(piecewise_polynomial_part),
        K_(K),
        A_(A),
        alpha_(alpha),
        piecewise_polynomial_part_(piecewise_polynomial_part) {
    DRAKE_ASSERT(K.rows() == rows());
    DRAKE_ASSERT(K.cols() == A.rows());
    DRAKE_ASSERT(A.rows() == A.cols());
    DRAKE_ASSERT(alpha.rows() == A.cols());
    DRAKE_ASSERT(alpha.cols() ==
                 piecewise_polynomial_part.get_number_of_segments());
    DRAKE_ASSERT(piecewise_polynomial_part.rows() == rows());
    DRAKE_ASSERT(piecewise_polynomial_part.cols() == 1);
  }

  // from PiecewisePolynomial
  ExponentialPlusPiecewisePolynomial(
      const PiecewisePolynomial<T>& piecewise_polynomial_part);

  ~ExponentialPlusPiecewisePolynomial() override = default;

  std::unique_ptr<Trajectory<T>> Clone() const override;

  MatrixX<T> value(const T& t) const override;

  ExponentialPlusPiecewisePolynomial derivative(int derivative_order = 1) const;

  std::unique_ptr<Trajectory<T>> MakeDerivative(
      int derivative_order = 1) const override {
    return derivative(derivative_order).Clone();
  };

  Eigen::Index rows() const override;

  Eigen::Index cols() const override;

  void shiftRight(double offset);

 private:
  MatrixX<T> K_;
  MatrixX<T> A_;
  MatrixX<T> alpha_;
  PiecewisePolynomial<T> piecewise_polynomial_part_;
};

}  // namespace trajectories
}  // namespace drake
