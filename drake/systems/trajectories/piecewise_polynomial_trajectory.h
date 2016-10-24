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
  const PiecewisePolynomial<double> pp_;
};

}  // namespace drake
