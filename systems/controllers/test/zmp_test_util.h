#pragma once

#include <vector>

#include "drake/systems/controllers/zmp_planner.h"

namespace drake {
namespace systems {
namespace controllers {

/** A structure for storing trajectories from simulating a linear inverted
 * pendulum model (LIPM) using the policy from a ZMPPlanner.
 */
struct ZMPTestTraj {
  explicit ZMPTestTraj(int N) {
    time.resize(N);
    nominal_com.resize(6, N);
    desired_zmp.resize(2, N);
    x.resize(4, N);
    u.resize(2, N);
    cop.resize(2, N);
  }
  Eigen::Matrix<double, 1, Eigen::Dynamic> time;
  Eigen::Matrix<double, 6, Eigen::Dynamic> nominal_com;
  Eigen::Matrix<double, 2, Eigen::Dynamic> desired_zmp;

  Eigen::Matrix<double, 4, Eigen::Dynamic> x;
  Eigen::Matrix<double, 2, Eigen::Dynamic> u;
  Eigen::Matrix<double, 2, Eigen::Dynamic> cop;
};

/**
 * Forward simulation using the linear policy from `zmp_planner` starting from
 * the initial condition `x0` using explicit Euler integration.
 * @param zmp_planner, Contains planned center of mass trajectory,
 * value function, and linear policy.
 * @param x0, Initial condition.
 * @param dt, Time step.
 * @param extra_time_at_the_end, Simulate `extra_time_at_the_end` seconds past
 * the end of the trajectories for convergence.
 * @return ZMPTestTraj that contains all the information.
 */
ZMPTestTraj SimulateZMPPolicy(const ZMPPlanner& zmp_planner,
                              const Eigen::Vector4d& x0, double dt,
                              double extra_time_at_the_end);

/**
 * Generates desired ZMP trajectories as piecewise polynomials given the
 * desired footsteps. The knot points are generated as follows:
 * <pre>
 * T: 0, knots: fs[0]
 * T: ss, knots: fs[0]
 * T: ss + ds, knots: fs[1]
 * T: 2 * ss + ds, knots: fs[1]
 * T: 2 * ss + 2 * ds, knots: fs[2]
 * T: 3 * ss + 2 * ds, knots: fs[2]
 *  ...
 * </pre>
 * ss stands for `single_support_duration`,
 * and ds for `double_support_duration`.
 * @param footsteps, X Y pair
 * @param double_support_duration, Duration for double support.
 * @param single_support_duration, Duration for single support.
 * @return Three trajectories: 0 is zero-order-hold, 1 is linear, 2 is cubic.
 */
std::vector<trajectories::PiecewisePolynomial<double>> GenerateDesiredZMPTrajs(
    const std::vector<Eigen::Vector2d>& footsteps,
    double double_support_duration, double single_support_duration);

}  // namespace controllers
}  // namespace systems
}  // namespace drake
