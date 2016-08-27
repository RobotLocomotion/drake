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
    const std::vector<double>& time_stamps)
    : time_stamps_(time_stamps), joint_trajectories_(joint_trajectories) {
  DRAKE_ASSERT(time_stamps_.size() == joint_trajectories.cols());
  num_points_ = time_stamps.size();
}

void PolynomialTrajectoryFitGenerator::ResetTrajectory(
    const MatrixXd &joint_trajectories,
    const std::vector<double> &time_stamps) {
  time_stamps_.clear();
  time_stamps_ = time_stamps;
  joint_trajectories_ = joint_trajectories;
}

PiecewisePolynomial<double>
PolynomialTrajectoryFitGenerator::GenerateTrajectoryPolynomial() {
  vector<PolynomialTrajectoryFitGenerator::PPMatrix> polys;

  // For each timestep, create a PolynomialMatrix for each joint
  // position.  Each column of joint_trajectories_ represents a particular time,
  // and the rows of that column contain values for each joint
  // coordinate.
  for (int i = 0; i < num_points_; i++) {
    PolynomialTrajectoryFitGenerator::PPMatrix poly_matrix(
        joint_trajectories_.rows(), 1);
    const auto traj_now = joint_trajectories_.col(i);

    // Produce interpolating polynomials for each joint coordinate.
    for (int row = 0; row < joint_trajectories_.rows(); row++) {
      Eigen::Vector2d coeffs(0, 0);
      coeffs[0] = traj_now(row);
      if (i != num_points_ - 1) {
        // Set the coefficient such that it will reach the value of
        // the next timestep at the time when we advance to the next
        // piece.  In the event that we're at the end of the
        // trajectory, this will be left 0.
        coeffs[1] = (joint_trajectories_(row, i + 1) - coeffs[0]) /
                    (time_stamps_[i + 1] - time_stamps_[i]);
      }
      poly_matrix(row) = PolynomialTrajectoryFitGenerator::PPPoly(coeffs);
    }
    polys.push_back(poly_matrix);
  }

  PolynomialTrajectoryFitGenerator::PPType pp_traj(polys, time_stamps_);
  return (pp_traj);
}

}  // namespace kuka_iia_arm
}  // namespace examples
}  // namespace drake
