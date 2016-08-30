#include "polynomial_trajectory_fit_generator.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::VectorXd;
using std::vector;

// Helper typedefs for the methods in this class.
typedef PiecewisePolynomial<double> PPType;
typedef PPType::PolynomialType PPPoly;
typedef PPType::PolynomialMatrix PPMatrix;

// TODO(naveenoid): This method is very similar to
// PiecewisePolynomial::FirstOrderHold except for type compatibility issues.
// Consider finding a way to wrap that method instead.
PiecewisePolynomial<double> PolynomialTrajectoryFitGenerator(
    const Eigen::MatrixXd& joint_trajectories,
    const std::vector<double>& segment_times) {
  DRAKE_ASSERT(segment_times.size() == joint_trajectories.cols());
  DRAKE_ASSERT(segment_times.size() == segment_times.size());

  int num_points = segment_times.size();
  vector<PPMatrix> polys;

  // For each timestamp, create a PolynomialMatrix for each joint position. Each
  // column of joint_trajectories represents a particular time, and and each
  // row contains a reference position or velocity for a joint DOF.
  for (int i = 0; i < num_points; ++i) {
    PPMatrix poly_matrix(
        joint_trajectories.rows(), 1);
    const auto traj_now = joint_trajectories.col(i);

    // Produces interpolating polynomials for each joint coordinate.
    // TODO(naveenoid): Consider redesign to accept user defined polynomial
    // order.
    for (int row = 0; row < joint_trajectories.rows(); ++row) {
      Eigen::Vector2d coeffs(0, 0);
      coeffs[0] = traj_now(row);
      if (i != num_points - 1) {
        // Sets the coefficient such that it will reach the value of the next
        // timestamp at the time when we advance to the next trajectory
        // segment. In the event that we're at the end of the trajectory,
        // this value will be left as 0. Basically this performs a first order
        // interpolation of the value between the current and next
        // segment_times.
        coeffs[1] = (joint_trajectories(row, i + 1) - coeffs[0]) /
                    (segment_times[i + 1] - segment_times[i]);
      }
      poly_matrix(row) = PPPoly(coeffs);
    }
    polys.push_back(poly_matrix);
  }

  PiecewisePolynomial<double> pp_traj(polys, segment_times);
  return (pp_traj);
}

}  // namespace kuka_iia_arm
}  // namespace examples
}  // namespace drake
