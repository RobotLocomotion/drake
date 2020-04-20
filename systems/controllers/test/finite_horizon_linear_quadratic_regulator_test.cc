#include "drake/systems/controllers/finite_horizon_linear_quadratic_regulator.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/primitives/linear_system.h"
#include "drake/systems/primitives/symbolic_vector_system.h"

namespace drake {
namespace systems {
namespace controllers {
namespace {

// For a time-invariant system and cost, the LinearQuadraticRegulator should be
// a stable fixed-point for the finite-horizon solution.
GTEST_TEST(FiniteHorizonLQRTest, InfiniteHorizonTest) {
  // Double integrator dynamics: qddot = u, where q is the position coordinate.
  Eigen::Matrix2d A;
  Eigen::Vector2d B;
  A << 0, 1, 0, 0;
  B << 0, 1;
  LinearSystem<double> sys(A, B, Eigen::Matrix<double, 0, 2>::Zero(),
                           Eigen::Matrix<double, 0, 1>::Zero());

  Eigen::Matrix2d Q = Eigen::Matrix2d::Identity();
  Vector1d R = Vector1d(1.0);

  LinearQuadraticRegulatorResult lqr_result =
      LinearQuadraticRegulator(A, B, Q, R, Eigen::Matrix<double, 0, 0>());

  const double t0 = 0;
  const double tf = 8.0;
  FiniteHorizonLinearQuadraticRegulatorOptions options;
  auto context = sys.CreateDefaultContext();
  context->FixInputPort(0, Vector1d(0.0));

  // Test that it converges towards the fixed point from zero final cost.
  FiniteHorizonLinearQuadraticRegulatorResult result =
      FiniteHorizonLinearQuadraticRegulator(sys, *context, t0, tf, Q, R,
                                            options);
  EXPECT_TRUE(result.S.start_time() == t0);
  EXPECT_TRUE(result.S.end_time() == tf);
  // Confirm that it's initialized to zero.
  EXPECT_TRUE(result.S.value(tf).isZero(1e-12));
  // Confirm that it converges to the solution.
  EXPECT_TRUE(CompareMatrices(result.S.value(t0), lqr_result.S, 1e-5));
  EXPECT_TRUE(result.K.start_time() == t0);
  EXPECT_TRUE(result.K.end_time() == tf);
  EXPECT_TRUE(CompareMatrices(result.K.value(t0), lqr_result.K, 1e-5));

  // Test that it stays at the fixed-point if initialized at the fixed point.
  options.Qf = lqr_result.S;
  result = FiniteHorizonLinearQuadraticRegulator(sys, *context, t0, t0 + 0.1, Q,
                                                 R, options);
  EXPECT_TRUE(CompareMatrices(result.S.value(t0), lqr_result.S, 1e-12));
  EXPECT_TRUE(CompareMatrices(result.K.value(t0), lqr_result.K, 1e-12));
}

// Verify that we can stabilize a non-zero fixed-point specified via the
// nominal trajectory options.
GTEST_TEST(FiniteHorizonLQRTest, NominalTrajectoryTest) {
  symbolic::Variable x("x");
  symbolic::Variable u("u");
  const auto system = SymbolicVectorSystemBuilder()
                          .state(x)
                          .input(u)
                          .dynamics(-x + pow(x, 3) + u)
                          .Build();
  auto context = system->CreateDefaultContext();
  context->FixInputPort(0, Vector1d(0));

  const Vector1d Q(1.0);
  const Vector1d R(1.0);
  FiniteHorizonLinearQuadraticRegulatorOptions options;

  // We can make any state a fixed point using u = x - x^3.
  for (const double x0 : std::vector<double>({-2, -1., 0., 1, 2})) {
    const Vector1d x0v(x0);
    const Vector1d u0v(x0 - std::pow(x0, 3));
    context->SetContinuousState(x0v);
    context->FixInputPort(0, u0v);

    auto linear_sys = Linearize(*system, *context);
    LinearQuadraticRegulatorResult lqr_result = LinearQuadraticRegulator(
        linear_sys->A(), linear_sys->B(), Q, R, Eigen::Matrix<double, 0, 0>());

    // Test that it stays at the fixed-point if initialized at the fixed point.
    const double t0 = 2;
    const double tf = 2.3;
    options.Qf = lqr_result.S;
    trajectories::PiecewisePolynomial<double> x0_traj(x0v);
    options.x0 = &x0_traj;
    trajectories::PiecewisePolynomial<double> u0_traj(u0v);
    options.u0 = &u0_traj;
    FiniteHorizonLinearQuadraticRegulatorResult result =
        FiniteHorizonLinearQuadraticRegulator(*system, *context, t0, tf,
                                              Q, R, options);
    EXPECT_TRUE(
        CompareMatrices(result.S.value(t0), result.S.value(tf), 1e-12));
    EXPECT_TRUE(CompareMatrices(result.S.value(t0), lqr_result.S, 1e-12));
    EXPECT_TRUE(CompareMatrices(result.K.value(t0), lqr_result.K, 1e-12));
  }
}

}  // namespace
}  // namespace controllers
}  // namespace systems
}  // namespace drake
