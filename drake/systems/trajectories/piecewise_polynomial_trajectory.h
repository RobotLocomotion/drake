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
template <typename CoefficientType = double>
class PiecewisePolynomialTrajectory : public Trajectory {
 public:
  explicit PiecewisePolynomialTrajectory(const PiecewisePolynomial<CoefficientType>& pp)
      : pp_(pp) {}

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> value(
      double t) const override {
    return pp_.value(t);
  }

  Eigen::Index length() const { return pp_.rows(); }

 private:
  const PiecewisePolynomial<double> pp_;
};

}  // namespace drake
