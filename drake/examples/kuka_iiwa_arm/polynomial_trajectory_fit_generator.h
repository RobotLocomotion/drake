#pragma once

#include <iostream>

#include "drake/common/polynomial.h"
#include "drake/systems/trajectories/PiecewisePolynomial.h"
#include "drake/systems/vector.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

/// This class converts an input trajectory into a PiecewisePolynomial.
class PolynomialTrajectoryFitGenerator {
 public:
  /// Helper typedefs for the methods of this class.
  typedef PiecewisePolynomial<double> PPType;
  typedef PPType::PolynomialType PPPoly;
  typedef PPType::PolynomialMatrix PPMatrix;

  PolynomialTrajectoryFitGenerator(const MatrixXd& joint_trajectories,
                                   const std::vector<double>& time_stamps);

  /// Generates a PiecewisePolynomial corresponding to the joint
  /// trajectory and time stamps passed to this class. The dimensionality
  /// of the polynomial is the same as that of the joint trajectory, i.e.
  /// same as the number of degrees of freedom in the rigid body tree.
  PiecewisePolynomial<double> GenerateTrajectoryPolynomial();

  void ResetTrajectory(const MatrixXd& joint_trajectories,
                       const std::vector<double>& time_stamps);

 private:
  std::vector<double> time_stamps_;
  int num_points_;
  MatrixXd joint_trajectories_;
};

}
}
}
