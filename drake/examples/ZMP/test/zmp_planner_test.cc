#include "drake/common/eigen_matrix_compare.h"
#include "drake/examples/ZMP/zmp_test_util.h"

#include "drake/systems/controllers/zmp_planner.h"

#include "gtest/gtest.h"

namespace drake {
namespace examples {
namespace zmp {

// Given the value function of the form:
// J(x_bar) = x_bar^T * S1 * x_bar + x_bar^T * S2 + constant,
// where x_bar = [com - zmp_d_tf; comd],
// the optimal control u (CoM acceleration) is achieved by:
// min_u L(y, u) + dV / dx_bar * x_bar_dot,
// where L = (y - y_d)^T * Qy * (y - y_d) + u^T * R u.
// The minimum is achived when the derivative w.r.t u equals to 0.
// This expands to:
// 2 * (C * x + D * u - y_d)^T * Qy * D + 2 * u^T * R +
// (2 * S1 * x_bar + S2)^T * B = 0.
// If we collect the terms,
// (R + D^T * Qy * D) * u =
// -(C * x - y_d)^T * Qy * D - (S1 * x_bar + 0.5 * S2)^T * B
Eigen::Vector2d ComputeOptimalCoMddGivenValueFunction(
    const drake::systems::ZMPPlanner& zmp_planner, double time,
    const Eigen::Vector4d& x) {
  Eigen::Vector4d x_bar = x;
  x_bar.head<2>() -= zmp_planner.get_final_desired_zmp();
  Eigen::Vector2d zmp_d = zmp_planner.get_desired_zmp(time);

  const Eigen::Matrix<double, 4, 2>& B = zmp_planner.get_B();
  const Eigen::Matrix<double, 2, 4>& C = zmp_planner.get_C();
  const Eigen::Matrix<double, 2, 2>& D = zmp_planner.get_D();
  const Eigen::Matrix<double, 2, 2>& Qy = zmp_planner.get_Qy();
  const Eigen::Matrix<double, 2, 2>& R = zmp_planner.get_R();
  const Eigen::Matrix<double, 4, 4>& S =
      zmp_planner.get_value_function_second_derivative();
  Eigen::Vector4d s1 = zmp_planner.get_value_function_first_derivative(time);

  Eigen::Matrix2d R1 = R + D.transpose() * Qy * D;
  Eigen::Vector2d lin = -(C * x - zmp_d).transpose() * Qy * D -
                        (S * x_bar + 0.5 * s1).transpose() * B;
  Eigen::Vector2d comdd_d = R1.inverse() * lin;

  return comdd_d;
}

// Test the optimal policy can stabilize the CoM state around the nominal
// trajectory, and the optimal control and the value function are consistent
// with each other.
GTEST_TEST(TestZMP, TestOptimalControl) {
  std::vector<Eigen::Vector2d> footsteps = {
      Eigen::Vector2d(0, 0),    Eigen::Vector2d(0.5, 0.1),
      Eigen::Vector2d(1, -0.1), Eigen::Vector2d(1.5, 0.1),
      Eigen::Vector2d(2, -0.1), Eigen::Vector2d(2.5, 0)};

  std::vector<PiecewisePolynomial<double>> zmp_trajs =
      GenerateDesiredZMPTrajs(footsteps, 0.5, 1);

  for (const auto& zmp_traj : zmp_trajs) {
    double z = 1;
    Eigen::Vector4d x0(0, 0, 0, 0);

    drake::systems::ZMPPlanner zmp_planner;
    zmp_planner.Plan(zmp_traj, x0, z);

    // Perturb initial condition.
    x0 << 0.1, -0.05, 0.1, 0.1;

    // Simulate.
    double sample_dt = 0.01;
    ZMPTestTraj result = Simulate(zmp_planner, x0, sample_dt, 2);

    // Test the optimal control computed differently.
    for (int i = 0; i < result.time.size(); i++) {
      Eigen::Vector2d u0 = ComputeOptimalCoMddGivenValueFunction(
          zmp_planner, result.time[i], result.x.col(i));
      EXPECT_TRUE(CompareMatrices(u0, result.u.col(i), 1e-8,
                                  MatrixCompareType::absolute));
    }

    int N = result.time.size();
    // Expect the trajectory converges to the desired at the end.
    EXPECT_TRUE(CompareMatrices(result.x.col(N - 1).head<4>(),
                                result.nominal_com.col(N - 1).head<4>(), 1e-4,
                                MatrixCompareType::absolute));
  }
}

}  // namespace zmp
}  // namespace examples
}  // namespace drake
