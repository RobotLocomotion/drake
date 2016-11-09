#include "drake/systems/trajectories/piecewise_polynomial_trajectory.h"

#include <vector>

using Eigen::MatrixXd;

namespace drake {

PiecewisePolynomialTrajectory::PiecewisePolynomialTrajectory(
  const MatrixXd& trajectory_matrix, const std::vector<double>& times) {
  typedef PiecewisePolynomial<double> PPType;

  std::vector<PPType::PolynomialMatrix> polys;
  std::vector<double> segment_times;
  const int num_time_steps = times.size();
  // For each timestep, creates a PolynomialMatrix for each joint position.
  // Each column of trajectory_matrix represents a particular time, and the
  // rows of that column contain values for each joint coordinate.
  for (int i = 0; i < num_time_steps; ++i) {
    PPType::PolynomialMatrix poly_matrix(trajectory_matrix.rows(), 1);
    const auto traj_now = trajectory_matrix.col(i);

    // Produces interpolating polynomials for each joint coordinate.
    if (i != num_time_steps - 1) {
      for (int row = 0; row < trajectory_matrix.rows(); ++row) {
        Eigen::Vector2d coeffs(0, 0);
        coeffs[0] = traj_now(row);
        // Sets the coefficient such that it will reach the value of
        // the next timestep at the time when we advance to the next
        // piece.  In the event that we're at the end of the
        // trajectory, this will be left 0.
        coeffs[1] = (trajectory_matrix(row, i + 1) - coeffs[0]) /
                    (times.at(i + 1) - times.at(i));
        poly_matrix(row) = PPType::PolynomialType(coeffs);
      }
      polys.push_back(poly_matrix);
    }
    segment_times.push_back(times.at(i));
  }
  pp_ = PPType(polys, segment_times);
}

}  // namespace drake
