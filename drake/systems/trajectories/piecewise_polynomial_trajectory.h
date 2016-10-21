#pragma once

#include <Eigen/Core>

#include "drake/common/drake_export.h"
#include "drake/systems/trajectories/PiecewisePolynomial.h"
#include "drake/systems/trajectories/trajectory.h"

namespace drake {

/**
 * A PiecewisePolynomialTrajectory is a Trajectory that is represented by
 * (implemented in terms of) a PiecewisePolynomial.
 */
class DRAKE_EXPORT PiecewisePolynomialTrajectory : public Trajectory {
 public:
  explicit PiecewisePolynomialTrajectory(
      const PiecewisePolynomial<double>& pp)
      : pp_(pp) {}

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> value(
      double t) const override {
    return pp_.value(t);
  }

  /**
   * @return The length of the output vector, which is also the number of rows
   * in the PiecewisePolynomial. If the output is a matrix, length() is the
   * number of rows in the matrix.
   */
  Eigen::Index length() const override { return pp_.rows(); }

 private:
  const PiecewisePolynomial<double> pp_;
};

}  // namespace drake
