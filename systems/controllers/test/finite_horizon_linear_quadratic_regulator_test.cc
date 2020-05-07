#include "drake/systems/controllers/finite_horizon_linear_quadratic_regulator.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"
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
  EXPECT_TRUE(result.S->start_time() == t0);
  EXPECT_TRUE(result.S->end_time() == tf);
  // Confirm that it's initialized to zero.
  EXPECT_TRUE(result.S->value(tf).isZero(1e-12));
  // Confirm that it converges to the solution.
  EXPECT_TRUE(CompareMatrices(result.S->value(t0), lqr_result.S, 1e-5));
  EXPECT_TRUE(result.K->start_time() == t0);
  EXPECT_TRUE(result.K->end_time() == tf);
  EXPECT_TRUE(CompareMatrices(result.K->value(t0), lqr_result.K, 1e-5));
  EXPECT_TRUE(CompareMatrices(result.x0->value(t0), Eigen::Vector2d::Zero()));
  EXPECT_TRUE(CompareMatrices(result.x0->value(tf), Eigen::Vector2d::Zero()));
  EXPECT_TRUE(CompareMatrices(result.u0->value(t0), Vector1d::Zero()));
  EXPECT_TRUE(CompareMatrices(result.u0->value(tf), Vector1d::Zero()));

  // Test that the System version also works.
  const std::unique_ptr<System<double>> regulator =
      MakeFiniteHorizonLinearQuadraticRegulator(sys, *context, t0, tf, Q, R,
                                                options);
  auto regulator_context = regulator->CreateDefaultContext();
  const Eigen::Vector2d x(.1, -.3);
  regulator_context->FixInputPort(0, x);
  EXPECT_EQ(regulator->get_input_port(0).size(), 2);
  EXPECT_EQ(regulator->get_output_port(0).size(), 1);
  EXPECT_TRUE(
      CompareMatrices(regulator->get_output_port(0).Eval(*regulator_context),
                      -lqr_result.K * x, 1e-5));

  // Test that it stays at the fixed-point if initialized at the fixed point.
  options.Qf = lqr_result.S;
  result = FiniteHorizonLinearQuadraticRegulator(sys, *context, t0, t0 + 0.1, Q,
                                                 R, options);
  EXPECT_TRUE(CompareMatrices(result.S->value(t0), lqr_result.S, 1e-12));
  EXPECT_TRUE(CompareMatrices(result.K->value(t0), lqr_result.K, 1e-12));
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
        FiniteHorizonLinearQuadraticRegulator(*system, *context, t0, tf, Q, R,
                                              options);
    EXPECT_TRUE(
        CompareMatrices(result.S->value(t0), result.S->value(tf), 1e-12));
    EXPECT_TRUE(CompareMatrices(result.S->value(t0), lqr_result.S, 1e-12));
    EXPECT_TRUE(CompareMatrices(result.K->value(t0), lqr_result.K, 1e-12));
    EXPECT_TRUE(CompareMatrices(result.x0->value(t0), x0v));
    EXPECT_TRUE(CompareMatrices(result.x0->value(tf), x0v));
    EXPECT_TRUE(CompareMatrices(result.u0->value(t0), u0v));
    EXPECT_TRUE(CompareMatrices(result.u0->value(tf), u0v));

    // Test that the System version also works.
    const std::unique_ptr<System<double>> regulator =
        MakeFiniteHorizonLinearQuadraticRegulator(*system, *context, t0, tf, Q,
                                                  R, options);
    auto regulator_context = regulator->CreateDefaultContext();
    const Vector1d xs(-.3);
    regulator_context->FixInputPort(0, xs);
    EXPECT_EQ(regulator->get_input_port(0).size(), 1);
    EXPECT_EQ(regulator->get_output_port(0).size(), 1);
    EXPECT_TRUE(
        CompareMatrices(regulator->get_output_port(0).Eval(*regulator_context),
                        u0v - lqr_result.K * (xs - x0v), 1e-5));
  }
}

// Ensures that we can scalar convert the System version of the regulator.
GTEST_TEST(FiniteHorizonLQRTest, ResultSystemIsScalarConvertible) {
  Eigen::Matrix2d A;
  Eigen::Vector2d B;
  A << 0, 1, 0, 0;
  B << 0, 1;
  LinearSystem<double> sys(A, B, Eigen::Matrix<double, 0, 2>::Zero(),
                           Eigen::Matrix<double, 0, 1>::Zero());
  auto context = sys.CreateDefaultContext();
  context->FixInputPort(0, Vector1d(0.0));
  const double t0 = 0.0;
  Eigen::Matrix2d Q = Eigen::Matrix2d::Identity();
  Vector1d R = Vector1d(1.0);

  const std::unique_ptr<System<double>> regulator =
      MakeFiniteHorizonLinearQuadraticRegulator(sys, *context, t0, t0 + 0.1, Q,
                                                R);

  EXPECT_TRUE(is_autodiffxd_convertible(*regulator, [&](const auto& converted) {
    EXPECT_EQ(converted.num_input_ports(), 1);
    EXPECT_EQ(converted.num_total_inputs(), 2);
    EXPECT_EQ(converted.num_output_ports(), 1);
    EXPECT_EQ(converted.num_total_outputs(), 1);
  }));
  // TODO(russt): Re-enable symbolic pending resolution of #12253:
  EXPECT_FALSE(is_symbolic_convertible(*regulator));
}

}  // namespace
}  // namespace controllers
}  // namespace systems
}  // namespace drake
