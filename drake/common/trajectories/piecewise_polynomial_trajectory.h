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
    return pp_.value(t);
  }

  /**
   * Takes the derivative of this PiecewisePolynomialTrajectory.
   * Each segment of the returned PiecewisePolynomialTrajectory is the
   * derivative of the segment in the original PiecewisePolynomialTrajectory.
   * @param derivative_order The number of times to take the derivative before
   * returning.
   * @return The nth derivative of this object.
   */
  std::unique_ptr<Trajectory> derivative(
      int derivative_order = 1) const override {
    return std::make_unique<PiecewisePolynomialTrajectory>(
        PiecewisePolynomialTrajectory(pp_.derivative(derivative_order)));
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
