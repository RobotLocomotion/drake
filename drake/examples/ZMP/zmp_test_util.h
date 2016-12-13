#pragma once

#include <vector>
#include "drake/systems/controllers/zmp_planner.h"

namespace drake {
namespace examples {
namespace zmp {

struct ZMPTestTraj {
  Eigen::Matrix<double, 1, Eigen::Dynamic> time;
  Eigen::Matrix<double, 6, Eigen::Dynamic> nominal_com;
  Eigen::Matrix<double, 2, Eigen::Dynamic> desired_zmp;

  Eigen::Matrix<double, 4, Eigen::Dynamic> x;
  Eigen::Matrix<double, 2, Eigen::Dynamic> u;
  Eigen::Matrix<double, 2, Eigen::Dynamic> cop;

  ZMPTestTraj(int N) {
    time.resize(N);
    nominal_com.resize(6, N);
    desired_zmp.resize(2, N);
    x.resize(4, N);
    u.resize(2, N);
    cop.resize(2, N);
  }
};

ZMPTestTraj Simulate(const drake::systems::ZMPPlanner& zmp_planner,
    const Eigen::Vector4d& x0, double dt, double T_final_offset);

// Generates desired ZMP trajectories (piecewise constant, linear, and cubic).
// The ZMP knot points moves forward in the x direction, and alternates in the
// y direction.
std::vector<PiecewisePolynomial<double>> GenerateDesiredZMPTrajs(
    const std::vector<Eigen::Vector2d>& footsteps,
    double double_support_duration,
    double single_support_duration);

}  // namespace zmp
}  // namespace examples
}  // namespace drake
