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
 Represents a piecewise-trajectory with piece @f$j@f$ given by:
 @f[
 x(t) = K e^{A (t - t_j)} \alpha_j + \sum_{i=0}^k \beta_{j,i}(t-t_j)^i,
 @f]
 where @f$k@f$ is the order of the `piecewise_polynomial_part` and @f$t_j@f$ is
 the start time of the @f$j@f$-th segment.

 This particular form can represent the solution to a linear dynamical system
 driven by a piecewise-polynomial input:
 @f[
 \dot{x}(t) = A x(t) + B u(t),
 @f]
 where the input @f$ u(t) @f$ is a piecewise-polynomial function of time. See
 [1] for details and a motivating use case.

 [1] R. Tedrake, S. Kuindersma, R. Deits and K. Miura, "A closed-form solution
 for real-time ZMP gait generation and feedback stabilization,"
 2015 IEEE-RAS 15th International Conference on Humanoid Robots (Humanoids),
 Seoul, 2015, pp. 936-940.

 @tparam_double_only
 */
template <typename T>
class ExponentialPlusPiecewisePolynomial final : public PiecewiseTrajectory<T> {
 public:
  // We are final, so this is okay.
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ExponentialPlusPiecewisePolynomial);

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
    DRAKE_ASSERT(K.rows() == this->rows());
    DRAKE_ASSERT(K.cols() == A.rows());
    DRAKE_ASSERT(A.rows() == A.cols());
    DRAKE_ASSERT(alpha.rows() == A.cols());
    DRAKE_ASSERT(alpha.cols() ==
                 piecewise_polynomial_part.get_number_of_segments());
    DRAKE_ASSERT(piecewise_polynomial_part.rows() == this->rows());
    DRAKE_ASSERT(piecewise_polynomial_part.cols() == 1);
  }

  // from PiecewisePolynomial
  ExponentialPlusPiecewisePolynomial(
      const PiecewisePolynomial<T>& piecewise_polynomial_part);

  ~ExponentialPlusPiecewisePolynomial() final;

  ExponentialPlusPiecewisePolynomial derivative(int derivative_order = 1) const;

  void shiftRight(double offset);

 private:
  // Trajectory overrides.
  std::unique_ptr<Trajectory<T>> DoClone() const final;
  MatrixX<T> do_value(const T& t) const final;
  Eigen::Index do_rows() const final;
  Eigen::Index do_cols() const final;

  MatrixX<T> K_;
  MatrixX<T> A_;
  MatrixX<T> alpha_;
  PiecewisePolynomial<T> piecewise_polynomial_part_;
};

}  // namespace trajectories
}  // namespace drake
