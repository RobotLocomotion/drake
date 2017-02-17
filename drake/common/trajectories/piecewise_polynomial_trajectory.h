#pragma once

#include <vector>

#include <Eigen/Core>

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/trajectory.h"

namespace drake {

/**
 * A PiecewisePolynomialTrajectory is a Trajectory that is represented by
 * (implemented in terms of) a PiecewisePolynomial.
 */
class PiecewisePolynomialTrajectory : public Trajectory {
 public:
  /**
   * Construct a PiecewisePolynomialTrajectory from a PiecewisePolynomial.
   */
  explicit PiecewisePolynomialTrajectory(const PiecewisePolynomial<double>& pp)
      : pp_(pp) {}

  /**
   * Evaluate this PiecewisePolynomial at a particular time.
   * @param t The time to evaluate.
   * @return a CoefficientMatrix that is the value of the wrapped
   * PiecewisePolynomial.
   */
  drake::MatrixX<double> value(double t) const override {
    DRAKE_DEMAND(pp_.value(t).cols() == 1);
    return pp_.value(t);
  }

  /**
   * Evaluates a derivative of the PiecewisePolynomialTrajectory at the given
   * time \p t.
   *
   * @param t The time at which to evaluate.
   * @param derivative_order If zero, return the value of the
   * Trajectory. Otherwise, take the derivative this many times before
   * evaluating.
   * @return The matrix of evaluated values.
   */
  drake::MatrixX<double> derivative(double t,
                                    int derivative_order = 1) const override {
    return pp_.derivative(t, derivative_order);
  }

  /**
   * @return The number of rows of the output vector, which is also
   * the number of rows in the PiecewisePolynomial.
   */
  Eigen::Index rows() const override { return pp_.rows(); }

  /**
   * @return The number of columns of the output vector, which is also
   * the number of columns in the PiecewisePolynomial.
   */
  Eigen::Index cols() const override { return pp_.cols(); }

 private:
  PiecewisePolynomial<double> pp_;
};

}  // namespace drake
