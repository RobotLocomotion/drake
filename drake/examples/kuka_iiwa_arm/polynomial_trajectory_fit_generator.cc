#include "polynomial_trajectory_fit_generator.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector2d;
using std::vector;

PolynomialTrajectoryFitGenerator::PolynomialTrajectoryFitGenerator(
    const Eigen::MatrixXd& joint_trajectories,
    const std::vector<double>& timestamps) {
  ResetTrajectory(joint_trajectories, timestamps);
}

void PolynomialTrajectoryFitGenerator::ResetTrajectory(
    const MatrixXd &joint_trajectories,
    const std::vector<double> &timestamps) {
  timestamps_ = timestamps;
  joint_trajectories_ = joint_trajectories;
  num_points_ = timestamps_.size();
  DRAKE_ASSERT(timestamps.size() == joint_trajectories.cols());
  DRAKE_ASSERT(timestamps.size() == timestamps_.size());
}

PiecewisePolynomial<double>
PolynomialTrajectoryFitGenerator::GenerateTrajectoryPolynomial() {
  vector<PolynomialTrajectoryFitGenerator::PPMatrix> polys;

  // For each timestamp, create a PolynomialMatrix for each joint position. Each
  // column of joint_trajectories_ represents a particular time, and and each
  // row contains a reference position or velocity for a joint DOF.
  for (int i = 0; i < num_points_; ++i) {
    PolynomialTrajectoryFitGenerator::PPMatrix poly_matrix(
        joint_trajectories_.rows(), 1);
    const auto traj_now = joint_trajectories_.col(i);

    // Produces interpolating polynomials for each joint coordinate.
    // TODO(naveenoid) :consider redesign to accept user defined polynomial
    // order.
    for (int row = 0; row < joint_trajectories_.rows(); ++row) {
      Eigen::Vector2d coeffs(0, 0);
      coeffs[0] = traj_now(row);
      if (i != num_points_ - 1) {
        // Sets the coefficient such that it will reach the value of the next
        // timestamp at the time when we advance to the next segment of the
        // trajectory. In the event that we're at the end of the trajectory,
        // this value will be left as 0. Basically this performs a first order
        // interpolate of the value between current and next timestamps.
        coeffs[1] = (joint_trajectories_(row, i + 1) - coeffs[0]) /
                    (timestamps_[i + 1] - timestamps_[i]);
      }
      poly_matrix(row) = PolynomialTrajectoryFitGenerator::PPPoly(coeffs);
    }
    polys.push_back(poly_matrix);
  }

  PolynomialTrajectoryFitGenerator::PPType pp_traj(polys, timestamps_);
  return (pp_traj);
}

}  // namespace kuka_iia_arm
}  // namespace examples
}  // namespace drake
