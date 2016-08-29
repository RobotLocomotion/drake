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

/// This class converts an input trajectory (a sequence of via-points) into a
/// PiecewisePolynomial.
 class DRAKEKUKAIIWAARM_EXPORT PolynomialTrajectoryFitGenerator {
 public:
  /// Helper typedefs for the methods in this class.
  typedef PiecewisePolynomial<double> PPType;
  typedef PPType::PolynomialType PPPoly;
  typedef PPType::PolynomialMatrix PPMatrix;

  PolynomialTrajectoryFitGenerator(const MatrixXd& joint_trajectories,
                                   const std::vector<double>& timestamps);

  /// Generates a PiecewisePolynomial corresponding to the joint trajectory and
  /// timestamps passed to this class. The dimensionality of the polynomial is
  /// the same as that of the joint trajectory, i.e. same as the number of
  ///  degrees of freedom in the rigid body tree. In the current implementation
  ///  polynomial order is one at most.
  PiecewisePolynomial<double> GenerateTrajectoryPolynomial();

  /// Sets or resets the stored joint_trajectory_ and timestamps_ variables.
  /// The number of timestamps (length of the timestamps std::vector) must
  /// equal the number of columns on the provided joint_trajectories.
  void ResetTrajectory(const MatrixXd& joint_trajectories,
                       const std::vector<double>& timestamps);

 private:

  // The trajectory to be fit is provided as a Eigen::MatrixXd containing via-
  // points and the std::vector timestamps_ encodes the timestamps at which
  // those via-points are specified.
  std::vector<double> timestamps_;
  int num_points_;
  MatrixXd joint_trajectories_;
};

}  // namespace kuka_iia_arm
}  // namespace examples
}  // namespace drake
