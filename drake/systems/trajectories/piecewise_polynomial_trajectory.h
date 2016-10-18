#pragma once

#include <Eigen/Core>

#include "drake/common/drake_export.h"
#include "trajectory.h"
#include "PiecewisePolynomial.h"

namespace drake {

template <typename CoefficientType = double>
class DRAKE_EXPORT PiecewisePolynomialTrajectory : public Trajectory {
 public:
  PiecewisePolynomialTrajectory(
      const PiecewisePolynomial<CoefficientType>& pp) : pp_(pp) {}

  virtual Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> value(
      double t) const {
    return pp_.value(t);
  }

  virtual Eigen::Index rows() const { return pp_.rows(); }

private:
  const PiecewisePolynomial<double> pp_;
};

}  // namespace drake