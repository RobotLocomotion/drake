#include "polynomial_trajectory_fit_generator.h"

#include "drake/common/drake_assert.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

typedef PiecewisePolynomial<double> PPType;
typedef PPType::PolynomialType PPPoly;
typedef PPType::PolynomialMatrix PPMatrix;

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector2d;
using std::vector;


PiecewisePolynomial<double> PolynomialTrajectoryFitGenerator(
    const Eigen::MatrixXd& joint_trajectories,
    const std::vector<double>& timestamps) {

  int num_points = timestamps.size();
  DRAKE_ASSERT(timestamps.size() == joint_trajectories.cols());
  vector<PPMatrix> polys;

  // For each timestamp, create a PolynomialMatrix for each joint position. Each
  // column of joint_trajectories represents a particular time, and and each
  // row contains a reference position or velocity for a joint DOF.
  for (int i = 0; i < num_points; ++i) {
    PPMatrix poly_matrix(
        joint_trajectories.rows(), 1);
    const auto traj_now = joint_trajectories.col(i);

    // Produces interpolating polynomials for each joint coordinate.
    // TODO(naveenoid) :consider redesign to accept user defined polynomial
    // order.
    for (int row = 0; row < joint_trajectories.rows(); ++row) {
      Eigen::Vector2d coeffs(0, 0);
      coeffs[0] = traj_now(row);
      if (i != num_points - 1) {
        // Sets the coefficient such that it will reach the value of the next
        // timestamp at the time when we advance to the next segment of the
        // trajectory. In the event that we're at the end of the trajectory,
        // this value will be left as 0. Basically this performs a first order
        // interpolate of the value between current and next timestamps.
        coeffs[1] = (joint_trajectories(row, i + 1) - coeffs[0]) /
                    (timestamps[i + 1] - timestamps[i]);
      }
      poly_matrix(row) = PPPoly(coeffs);
    }
    polys.push_back(poly_matrix);
  }

  PPType pp_traj(polys, timestamps);
  return (pp_traj);
}

}  // namespace kuka_iia_arm
}  // namespace examples
}  // namespace drake
