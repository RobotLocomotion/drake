#pragma once

#include <Eigen/Core>
#include <vector>

#include "drake/systems/trajectories/PiecewisePolynomial.h"
#include "drake/systems/trajectories/trajectory.h"

namespace drake {

/**
 * A PiecewisePolynomialTrajectory is a Trajectory that is represented by
 * (implemented in terms of) a PiecewisePolynomial.
 */
class PiecewisePolynomialTrajectory : public Trajectory {
 public:
  /**
   * Construct a PiecewisePolynomialTrajectory from a trajectory matrix.
   * @param trajectory_matrix Each column represents a particular
   * time, and the rows of that column contain values for each joint coordinate.
   * @param times Knot points: the times where the polynomial pieces connect.
   */
  explicit PiecewisePolynomialTrajectory(
      const Eigen::MatrixXd& trajectory_matrix,
      const std::vector<double>& times);

  /**
   * Construct a PiecewisePolynomialTrajectory from a PiecewisePolynomial.
   */
  explicit PiecewisePolynomialTrajectory(
      const PiecewisePolynomial<double>& pp)
      : pp_(pp) {}

  /**
   * Evaluate this PiecewisePolynomial at a particular time.
   * @param t The time to evaluate.
   * @return a CoefficientMatrix that is the value of the wrapped
   * PiecewisePolynomial.
   */
  PiecewisePolynomial<double>::CoefficientMatrix
  value(double t) const override {
    return pp_.value(t);
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
