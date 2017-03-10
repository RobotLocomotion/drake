#pragma once

#include <memory>
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

  std::unique_ptr<Trajectory> Clone() const override {
    return std::make_unique<PiecewisePolynomialTrajectory>(pp_);
  }

  /**
   * Evaluate this PiecewisePolynomial at a particular time.
   * @param t The time to evaluate.
   * @return a Matrix that is the value of the wrapped
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
        pp_.derivative(derivative_order));
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


  double get_start_time() const override { return pp_.getStartTime(); }

  double get_end_time() const override { return pp_.getEndTime(); }

  /**
   * @return A reference to the underlying piecewise polynomial.
   */
  const PiecewisePolynomial<double>& get_piecewise_polynomial() const {
    return pp_;
  }

 private:
  PiecewisePolynomial<double> pp_;
};

}  // namespace drake
