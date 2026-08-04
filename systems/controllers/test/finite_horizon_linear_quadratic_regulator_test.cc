#include "drake/systems/controllers/finite_horizon_linear_quadratic_regulator.h"

#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/trajectories/discrete_time_trajectory.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/framework/diagram_builder.h"
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
  Vector1d R = Vector1d(4.12);
  Eigen::Vector2d N(2.2, 1.3);

  LinearQuadraticRegulatorResult lqr_result =
      LinearQuadraticRegulator(A, B, Q, R, N);

  const double t0 = 0;
  const double tf = 40.0;
  FiniteHorizonLinearQuadraticRegulatorOptions options;
  auto context = sys.CreateDefaultContext();
  sys.get_input_port().FixValue(context.get(), 0.0);
  options.N = N;

  // Test that it converges towards the fixed point from zero final cost.
  FiniteHorizonLinearQuadraticRegulatorResult result =
      FiniteHorizonLinearQuadraticRegulator(sys, *context, t0, tf, Q, R,
                                            options);
  EXPECT_EQ(result.S->start_time(), t0);
  EXPECT_EQ(result.S->end_time(), tf);
  EXPECT_EQ(result.sx->start_time(), t0);
  EXPECT_EQ(result.sx->end_time(), tf);
  EXPECT_EQ(result.s0->start_time(), t0);
  EXPECT_EQ(result.s0->end_time(), tf);
  // Confirm that it's initialized to zero.
  EXPECT_TRUE(result.S->value(tf).isZero(1e-12));
  EXPECT_TRUE(result.sx->value(tf).isZero(1e-12));
  EXPECT_TRUE(result.s0->value(tf).isZero(1e-12));
  // Confirm that it converges to the infinite-horizon solution.
  EXPECT_TRUE(CompareMatrices(result.S->value(t0), lqr_result.S, 1e-5));
  EXPECT_TRUE(result.sx->value(t0).isZero(1e-12));
  EXPECT_TRUE(result.s0->value(t0).isZero(1e-12));
  EXPECT_EQ(result.K->start_time(), t0);
  EXPECT_EQ(result.K->end_time(), tf);
  EXPECT_TRUE(CompareMatrices(result.K->value(t0), lqr_result.K, 1e-5));
  EXPECT_TRUE(result.k0->value(t0).isZero(1e-12));
  EXPECT_TRUE(CompareMatrices(result.x0->value(t0), Eigen::Vector2d::Zero()));
  EXPECT_TRUE(CompareMatrices(result.x0->value(tf), Eigen::Vector2d::Zero()));
  EXPECT_TRUE(CompareMatrices(result.u0->value(t0), Vector1d::Zero()));
  EXPECT_TRUE(CompareMatrices(result.u0->value(tf), Vector1d::Zero()));

  // Test that the System version also works.
  const std::unique_ptr<System<double>> regulator =
      MakeFiniteHorizonLinearQuadraticRegulator(sys, *context, t0, tf, Q, R,
                                                options);
  auto regulator_context = regulator->CreateDefaultContext();
  const Eigen::Vector2d x(0.1, -0.3);
  regulator->get_input_port(0).FixValue(regulator_context.get(), x);
  EXPECT_EQ(regulator->get_input_port(0).size(), 2);
  EXPECT_EQ(regulator->get_output_port(0).size(), 1);
  EXPECT_TRUE(
      CompareMatrices(regulator->get_output_port(0).Eval(*regulator_context),
                      -lqr_result.K * x, 1e-5));

  // Test that it stays at the fixed-point if initialized at the fixed point.
  options.Qf = lqr_result.S;
  result = FiniteHorizonLinearQuadraticRegulator(sys, *context, t0, t0 + 2.0, Q,
                                                 R, options);
  EXPECT_TRUE(CompareMatrices(result.S->value(t0), lqr_result.S, 1e-12));
  EXPECT_TRUE(CompareMatrices(result.K->value(t0), lqr_result.K, 1e-12));
  // Already confirmed above that sx, s0, and k0 stay zero.

  // Test that the Square Root Method also maintains the fixed point.
  options.use_square_root_method = true;
  result = FiniteHorizonLinearQuadraticRegulator(sys, *context, t0, t0 + 2.0, Q,
                                                 R, options);
  EXPECT_TRUE(CompareMatrices(result.S->value(t0), lqr_result.S, 1e-4));
  EXPECT_TRUE(CompareMatrices(result.K->value(t0), lqr_result.K, 1e-4));
  EXPECT_TRUE(CompareMatrices(result.x0->value(t0), Eigen::Vector2d::Zero()));
  EXPECT_TRUE(CompareMatrices(result.x0->value(tf), Eigen::Vector2d::Zero()));
  EXPECT_TRUE(CompareMatrices(result.u0->value(t0), Vector1d::Zero()));
  EXPECT_TRUE(CompareMatrices(result.u0->value(tf), Vector1d::Zero()));
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
  system->get_input_port().FixValue(context.get(), 0.0);

  const Vector1d Q(1.0);
  const Vector1d R(1.0);
  FiniteHorizonLinearQuadraticRegulatorOptions options;

  // We can make any state a fixed point using u = x - x^3.
  for (const double x0 : std::vector<double>({-2, -1., 0., 1, 2})) {
    const Vector1d x0v(x0);
    const Vector1d u0v(x0 - std::pow(x0, 3));
    context->SetContinuousState(x0v);
    system->get_input_port().FixValue(context.get(), u0v);

    auto linear_sys = Linearize(*system, *context);
    LinearQuadraticRegulatorResult lqr_result = LinearQuadraticRegulator(
        linear_sys->A(), linear_sys->B(), Q, R, Eigen::Matrix<double, 0, 0>());

    // Test that it stays at the fixed-point if initialized at the fixed point.
    const double t0 = 2;
    const double tf = 2.3;
    options.Qf = lqr_result.S;
    trajectories::PiecewisePolynomial<double> x0_traj(x0v);
    options.x0 = &x0_traj;
    options.xd = &x0_traj;
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
    regulator->get_input_port(0).FixValue(regulator_context.get(), xs);
    EXPECT_EQ(regulator->get_input_port(0).size(), 1);
    EXPECT_EQ(regulator->get_output_port(0).size(), 1);
    EXPECT_TRUE(
        CompareMatrices(regulator->get_output_port(0).Eval(*regulator_context),
                        u0v - lqr_result.K * (xs - x0v), 1e-5));
  }
}

// Tests the affine terms by setting options.xd != options.x0.  We can stabilize
// the double integrator away from the origin, and the resulting solution should
// be match the quadratic form from LQR, but shifted to the new desired fixed
// point.
GTEST_TEST(FiniteHorizonLQRTest, DoubleIntegratorWithNonZeroGoal) {
  // Double integrator dynamics: qddot = u, where q is the position coordinate.
  Eigen::Matrix2d A;
  Eigen::Vector2d B;
  A << 0, 1, 0, 0;
  B << 0, 1;
  LinearSystem<double> sys(A, B, Eigen::Matrix<double, 0, 2>::Zero(),
                           Eigen::Matrix<double, 0, 1>::Zero());

  Eigen::Matrix2d Q = Eigen::Matrix2d::Identity();
  Vector1d R = Vector1d(4.12);

  LinearQuadraticRegulatorResult lqr_result =
      LinearQuadraticRegulator(A, B, Q, R);

  const double t0 = 0;
  const double tf = 15.0;
  FiniteHorizonLinearQuadraticRegulatorOptions options;
  auto context = sys.CreateDefaultContext();
  sys.get_input_port().FixValue(context.get(), 0.0);
  const Eigen::Vector2d xdv(2.87, 0);
  trajectories::PiecewisePolynomial<double> xd_traj(xdv);
  options.xd = &xd_traj;

  FiniteHorizonLinearQuadraticRegulatorResult result =
      FiniteHorizonLinearQuadraticRegulator(sys, *context, t0, tf, Q, R,
                                            options);
  // Confirm that it converges to the solution (the same quadratic form, but
  // shifted to the new origin): Sxx = lqr.S, sx = -Sxx*xd, s0 = xd'*Sxx*xd.
  EXPECT_TRUE(CompareMatrices(result.S->value(t0), lqr_result.S, 1e-5));
  EXPECT_TRUE(CompareMatrices(result.sx->value(t0), -lqr_result.S * xdv, 1e-5));
  EXPECT_TRUE(CompareMatrices(result.s0->value(t0),
                              xdv.transpose() * lqr_result.S * xdv, 1e-5));
  // The controller should be the same linear controller, but also shifted to
  // the new origin: Kx = lqr.K, k0 = -Kx*xdv.
  EXPECT_TRUE(CompareMatrices(result.K->value(t0), lqr_result.K, 1e-5));
  EXPECT_TRUE(CompareMatrices(result.k0->value(t0), -lqr_result.K * xdv, 1e-5));

  // Test that the System version also works.
  const std::unique_ptr<System<double>> regulator =
      MakeFiniteHorizonLinearQuadraticRegulator(sys, *context, t0, tf, Q, R,
                                                options);
  auto regulator_context = regulator->CreateDefaultContext();
  const Eigen::Vector2d x(0.1, -0.3);
  regulator->get_input_port(0).FixValue(regulator_context.get(), x);
  EXPECT_TRUE(
      CompareMatrices(regulator->get_output_port(0).Eval(*regulator_context),
                      -lqr_result.K * (x - xdv), 1e-5));
}

// Tests the affine terms by solving LQR from a different coordinate system.
// Given an affine system: xdot = Ax + Bu + c, with B invertible, we have a
// fixed point at x0=0, B*u0 = -c.  Normally, we would stabilize as a linear
// system in relative to x0, u0 using LQR.  Here set the coordinate system to
// (x0=0, u0=1), and set B*ud = -c.  The steady-state solution to the
// finite-horizon LQR problem will contain non-zero affine terms in order to get
// back to the offset form of this LQR controller.
GTEST_TEST(FiniteHorizonLQRTest, AffineSystemTest) {
  Eigen::Matrix2d A;
  Eigen::Matrix2d B;
  Eigen::Vector2d c;
  A << 0, 1, 0, 0;
  B << 5, 6, 7, 8;
  c << 9, 10;

  AffineSystem<double> sys(A, B, c, Eigen::Matrix<double, 0, 2>(),
                           Eigen::Matrix<double, 0, 2>(),
                           Eigen::Matrix<double, 0, 1>());

  Eigen::Matrix2d Q;
  Eigen::Matrix2d R;
  Eigen::Matrix2d N;
  Q << 0.8, 0.7, 0.7, 0.9;
  R << 1.4, 0.2, 0.2, 1.2;
  N << 0.1, 0.2, 0.3, 0.4;

  // Solve it again with the other interface to get access to S.
  LinearQuadraticRegulatorResult lqr_result =
      LinearQuadraticRegulator(A, B, Q, R, N);

  const double t0 = 0;
  const double tf = 70.0;
  FiniteHorizonLinearQuadraticRegulatorOptions options;
  auto context = sys.CreateDefaultContext();
  const Eigen::Vector2d u0v = Eigen::Vector2d::Ones();
  sys.get_input_port().FixValue(context.get(), u0v);
  const Eigen::Vector2d udv = -B.inverse() * c;
  trajectories::PiecewisePolynomial<double> ud_traj(udv);
  options.ud = &ud_traj;
  options.N = N;
  options.Qf = lqr_result.S;

  FiniteHorizonLinearQuadraticRegulatorResult result =
      FiniteHorizonLinearQuadraticRegulator(sys, *context, t0, tf, Q, R,
                                            options);
  // The LQR x'Sx is the correct solution, because this is equivalent to solving
  // the linear system in xbar=(x-x0), ubar=(u-0); and the LQR solution is
  // xbar'Sxbar, with x0=0.  However, in the finite-horizon version, all of the
  // affine terms must be correct to cancel each other out.
  EXPECT_TRUE(CompareMatrices(result.S->value(t0), lqr_result.S, 3.8e-5));
  EXPECT_TRUE(result.sx->value(t0).isZero(1e-5));
  EXPECT_TRUE(result.s0->value(t0).isZero(1e-5));
  // The LQR controller would be u = ud - Kx or u - u0 = -K(x-x0) - k0,
  // so Kx = lqr.K, k0 = -ud + u0.
  EXPECT_TRUE(CompareMatrices(result.K->value(t0), lqr_result.K, 4e-4));
  EXPECT_TRUE(CompareMatrices(result.k0->value(t0), -udv + u0v, 1e-5));

  // Test that the System version also works.
  const std::unique_ptr<System<double>> regulator =
      MakeFiniteHorizonLinearQuadraticRegulator(sys, *context, t0, tf, Q, R,
                                                options);
  auto regulator_context = regulator->CreateDefaultContext();
  const Eigen::Vector2d x(0.1, -0.3);
  regulator->get_input_port(0).FixValue(regulator_context.get(), x);
  EXPECT_TRUE(
      CompareMatrices(regulator->get_output_port(0).Eval(*regulator_context),
                      udv - lqr_result.K * x, 7.5e-5));

  // Test that the square root method also works.
  options.use_square_root_method = true;
  result = FiniteHorizonLinearQuadraticRegulator(sys, *context, t0, tf, Q, R,
                                                 options);
  EXPECT_TRUE(CompareMatrices(result.S->value(t0), lqr_result.S, 1e-4));
  EXPECT_TRUE(result.sx->value(t0).isZero(1e-5));
  EXPECT_TRUE(result.s0->value(t0).isZero(1e-5));
  EXPECT_TRUE(CompareMatrices(result.K->value(t0), lqr_result.K, 1e-4));
  EXPECT_TRUE(CompareMatrices(result.k0->value(t0), -udv + u0v, 1e-4));
}

// Ensures that the finite-horizon LQR works properly with a MultibodyPlant.
GTEST_TEST(FiniteHorizonLQRTest, MultibodyPlantTest) {
  using multibody::MultibodyPlant, multibody::Parser;

  auto multibody_plant = std::make_unique<MultibodyPlant<double>>(0.0);
  Parser(multibody_plant.get())
      .AddModelsFromUrl(
          "package://drake/examples/multibody/cart_pole/cart_pole.sdf");
  multibody_plant->Finalize();

  Eigen::VectorXd x0(4);
  x0 << 0, M_PI, 0, 0;
  Eigen::VectorXd u0(1);
  u0 << 0;
  auto context = multibody_plant->CreateDefaultContext();
  EXPECT_TRUE(context->has_only_continuous_state());
  multibody_plant->SetPositionsAndVelocities(context.get(), x0);
  multibody_plant->get_actuation_input_port().FixValue(context.get(), u0);

  const double t0 = 0.0;
  const double tf = 1.0;

  Eigen::DiagonalMatrix<double, 4> Q;
  Q.diagonal() << 10, 10, 1, 1;
  Eigen::DiagonalMatrix<double, 1> R;
  R.diagonal() << 0.01;

  FiniteHorizonLinearQuadraticRegulatorOptions options;
  options.Qf = Q;
  options.input_port_index =
      multibody_plant->get_actuation_input_port().get_index();

  auto finite_horizon_lqr = MakeFiniteHorizonLinearQuadraticRegulator(
      *multibody_plant, *context, t0, tf, Eigen::MatrixXd(Q),
      Eigen::MatrixXd(R), options);

  DiagramBuilder<double> builder;
  auto plant = builder.AddSystem(std::move(multibody_plant));
  auto controller = builder.AddSystem(std::move(finite_horizon_lqr));
  builder.Connect(controller->get_output_port(),
                  plant->get_actuation_input_port());
  builder.Connect(plant->get_state_output_port(), controller->get_input_port());

  auto diagram = builder.Build();
  Simulator simulator(*diagram);

  const double theta_init = M_PI * 0.9;
  Eigen::VectorXd x_init(4);
  x_init << 0, theta_init, 0, 0;
  simulator.get_mutable_context().SetContinuousState(x_init);

  simulator.AdvanceTo(tf);
  const double theta_final =
      simulator.get_context().get_continuous_state_vector()[1];
  // Check finite-horizon LQR controller regulates the cart-pole.
  const double theta_d = M_PI;
  EXPECT_LT(std::abs(theta_final - theta_d), std::abs(theta_init - theta_d));
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
  sys.get_input_port().FixValue(context.get(), 0.0);
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

  EXPECT_TRUE(is_symbolic_convertible(*regulator));
}

GTEST_TEST(FiniteHorizonLQRTest, SimulatorConfig) {
  // Double integrator dynamics: qddot = u, where q is the position coordinate.
  Eigen::Matrix2d A;
  Eigen::Vector2d B;
  A << 0, 1, 0, 0;
  B << 0, 1;
  LinearSystem<double> sys(A, B, Eigen::Matrix<double, 0, 2>::Zero(),
                           Eigen::Matrix<double, 0, 1>::Zero());

  Eigen::Matrix2d Q = Eigen::Matrix2d::Identity();
  Vector1d R = Vector1d(4.12);

  const double t0 = 0;
  const double tf = 3.0;
  FiniteHorizonLinearQuadraticRegulatorOptions options;
  options.simulator_config.use_error_control = false;
  options.simulator_config.max_step_size = 0.1;
  auto context = sys.CreateDefaultContext();
  sys.get_input_port().FixValue(context.get(), 0.0);

  FiniteHorizonLinearQuadraticRegulatorResult result =
      FiniteHorizonLinearQuadraticRegulator(sys, *context, t0, tf, Q, R,
                                            options);
  const trajectories::PiecewisePolynomial<double>* S =
      dynamic_cast<const trajectories::PiecewisePolynomial<double>*>(
          result.S.get());
  ASSERT_NE(S, nullptr);
  EXPECT_EQ(S->get_number_of_segments(), 30);

  options.simulator_config.max_step_size = 0.2;
  result = FiniteHorizonLinearQuadraticRegulator(sys, *context, t0, tf, Q, R,
                                                 options);
  S = dynamic_cast<const trajectories::PiecewisePolynomial<double>*>(
      result.S.get());
  ASSERT_NE(S, nullptr);
  EXPECT_EQ(S->get_number_of_segments(), 15);
}

// For a time-invariant system and cost, the
// DiscreteTimeLinearQuadraticRegulator should be a stable fixed-point for the
// discrete-time finite-horizon solution.
GTEST_TEST(DiscreteTimeFiniteHorizonLQRTest, InfiniteHorizonTest) {
  const double h = 0.1;
  // Discrete-time double integrator dynamics
  Eigen::Matrix2d A;
  Eigen::Vector2d B;
  A << 1, h, 0, 1;
  B << 0.5 * h * h, h;
  LinearSystem<double> sys(A, B, Eigen::Matrix<double, 0, 2>::Zero(),
                           Eigen::Matrix<double, 0, 1>::Zero(), h);

  Eigen::Matrix2d Q = Eigen::Matrix2d::Identity();
  Vector1d R = Vector1d(4.12);
  Eigen::Vector2d N(0.2, 1.3);

  LinearQuadraticRegulatorResult lqr_result =
      DiscreteTimeLinearQuadraticRegulator(A, B, Q, R, N);

  const double t0 = 0;
  const double tf = 40.0;
  FiniteHorizonLinearQuadraticRegulatorOptions options;
  auto context = sys.CreateDefaultContext();
  sys.get_input_port().FixValue(context.get(), 0.0);
  options.N = N;

  // Test that it converges towards the fixed point from zero final cost.
  FiniteHorizonLinearQuadraticRegulatorResult result =
      FiniteHorizonLinearQuadraticRegulator(sys, *context, t0, tf, Q, R,
                                            options);
  EXPECT_EQ(result.S->start_time(), t0);
  EXPECT_EQ(result.S->end_time(), tf);
  EXPECT_EQ(result.sx->start_time(), t0);
  EXPECT_EQ(result.sx->end_time(), tf);
  EXPECT_EQ(result.s0->start_time(), t0);
  EXPECT_EQ(result.s0->end_time(), tf);
  // Confirm that it's initialized to zero.
  EXPECT_TRUE(result.S->value(tf).isZero(1e-12));
  EXPECT_TRUE(result.sx->value(tf).isZero(1e-12));
  EXPECT_TRUE(result.s0->value(tf).isZero(1e-12));
  // Confirm that it converges to the infinite-horizon solution.
  EXPECT_TRUE(CompareMatrices(result.S->value(t0), lqr_result.S, 1e-5));
  EXPECT_TRUE(result.sx->value(t0).isZero(1e-12));
  EXPECT_TRUE(result.s0->value(t0).isZero(1e-12));
  EXPECT_EQ(result.K->start_time(), t0);
  EXPECT_DOUBLE_EQ(result.K->end_time(), tf - h);
  EXPECT_TRUE(CompareMatrices(result.K->value(t0), lqr_result.K, 1e-5));
  EXPECT_TRUE(result.k0->value(t0).isZero(1e-12));
  EXPECT_TRUE(CompareMatrices(result.x0->value(t0), Eigen::Vector2d::Zero()));
  EXPECT_TRUE(CompareMatrices(result.x0->value(tf), Eigen::Vector2d::Zero()));
  EXPECT_TRUE(CompareMatrices(result.u0->value(t0), Vector1d::Zero()));
  EXPECT_TRUE(CompareMatrices(result.u0->value(tf), Vector1d::Zero()));

  // Test that the System version also works.
  const std::unique_ptr<System<double>> regulator =
      MakeFiniteHorizonLinearQuadraticRegulator(sys, *context, t0, tf, Q, R,
                                                options);
  auto regulator_context = regulator->CreateDefaultContext();
  const Eigen::Vector2d x(0.1, -0.3);
  regulator->get_input_port(0).FixValue(regulator_context.get(), x);
  EXPECT_EQ(regulator->get_input_port(0).size(), 2);
  EXPECT_EQ(regulator->get_output_port(0).size(), 1);
  EXPECT_TRUE(
      CompareMatrices(regulator->get_output_port(0).Eval(*regulator_context),
                      -lqr_result.K * x, 1e-5));

  // Test that it stays at the fixed-point if initialized at the fixed point.
  options.Qf = lqr_result.S;
  result = FiniteHorizonLinearQuadraticRegulator(sys, *context, t0, t0 + 2.0, Q,
                                                 R, options);
  EXPECT_TRUE(CompareMatrices(result.S->value(t0), lqr_result.S, 1e-12));
  EXPECT_TRUE(CompareMatrices(result.K->value(t0), lqr_result.K, 1e-12));
  // Already confirmed above that sx, s0, and k0 stay zero.
}

// Tests the affine terms by setting options.xd != options.x0.  We can stabilize
// the double integrator away from the origin, and the resulting solution should
// be match the quadratic form from LQR, but shifted to the new desired fixed
// point.
GTEST_TEST(DiscreteTimeFiniteHorizonLQRTest,
           DiscreteTimeDoubleIntegratorWithNonZeroGoal) {
  const double h = 0.1;
  Eigen::Matrix2d A;
  Eigen::Vector2d B;
  A << 1, h, 0, 1;
  B << 0.5 * h * h, h;
  LinearSystem<double> sys(A, B, Eigen::Matrix<double, 0, 2>::Zero(),
                           Eigen::Matrix<double, 0, 1>::Zero(), h);

  Eigen::Matrix2d Q = Eigen::Matrix2d::Identity();
  Vector1d R = Vector1d(4.12);

  LinearQuadraticRegulatorResult lqr_result =
      DiscreteTimeLinearQuadraticRegulator(A, B, Q, R);

  const double t0 = 0;
  const double tf = 20.0;
  FiniteHorizonLinearQuadraticRegulatorOptions options;
  auto context = sys.CreateDefaultContext();
  sys.get_input_port().FixValue(context.get(), 0.0);
  const Eigen::Vector2d xdv(2.87, 0);
  trajectories::PiecewisePolynomial<double> xd_traj(xdv);
  options.xd = &xd_traj;

  FiniteHorizonLinearQuadraticRegulatorResult result =
      FiniteHorizonLinearQuadraticRegulator(sys, *context, t0, tf, Q, R,
                                            options);
  // Confirm that it converges to the solution (the same quadratic form, but
  // shifted to the new origin): Sxx = lqr.S, sx = -Sxx*xd, s0 = xd'*Sxx*xd.
  EXPECT_TRUE(CompareMatrices(result.S->value(t0), lqr_result.S, 1e-5));
  EXPECT_TRUE(CompareMatrices(result.sx->value(t0), -lqr_result.S * xdv, 1e-5));
  EXPECT_TRUE(CompareMatrices(result.s0->value(t0),
                              xdv.transpose() * lqr_result.S * xdv, 1e-5));
  // The controller should be the same linear controller, but also shifted to
  // the new origin: Kx = lqr.K, k0 = -Kx*xdv.
  EXPECT_TRUE(CompareMatrices(result.K->value(t0), lqr_result.K, 1e-5));
  EXPECT_TRUE(CompareMatrices(result.k0->value(t0), -lqr_result.K * xdv, 1e-5));

  // Test that the System version also works.
  const std::unique_ptr<System<double>> regulator =
      MakeFiniteHorizonLinearQuadraticRegulator(sys, *context, t0, tf, Q, R,
                                                options);
  auto regulator_context = regulator->CreateDefaultContext();
  const Eigen::Vector2d x(0.1, -0.3);
  regulator->get_input_port(0).FixValue(regulator_context.get(), x);
  EXPECT_TRUE(
      CompareMatrices(regulator->get_output_port(0).Eval(*regulator_context),
                      -lqr_result.K * (x - xdv), 1e-5));
}

// Tests the affine terms by solving LQR from a different coordinate system.
// Given an affine system: x[n+1] = Ax[n] + Bu + c, with B invertible, we have a
// fixed point at x0=0, B*u0 = -c.  Normally, we would stabilize as a linear
// system in relative to x0, u0 using LQR.  Here set the coordinate system to
// (x0=0, u0=1), and set B*ud = -c.  The steady-state solution to the
// finite-horizon LQR problem will contain non-zero affine terms in order to get
// back to the offset form of this LQR controller.
GTEST_TEST(DiscreteTimeFiniteHorizonLQRTest, AffineSystemTest) {
  const double time_period = 0.1;
  Eigen::Matrix2d A;
  Eigen::Matrix2d B;
  Eigen::Vector2d c;
  A << 0, 1, 0, 0;
  B << 5, 6, 7, 8;
  c << 9, 10;

  AffineSystem<double> sys(A, B, c, Eigen::Matrix<double, 0, 2>(),
                           Eigen::Matrix<double, 0, 2>(),
                           Eigen::Matrix<double, 0, 1>(), time_period);

  Eigen::Matrix2d Q;
  Eigen::Matrix2d R;
  Eigen::Matrix2d N;
  Q << 0.8, 0.7, 0.7, 0.9;
  R << 1.4, 0.2, 0.2, 1.2;
  N << 0.1, 0.2, 0.3, 0.4;

  // Solve it again with the other interface to get access to S.
  LinearQuadraticRegulatorResult lqr_result =
      DiscreteTimeLinearQuadraticRegulator(A, B, Q, R, N);

  const double t0 = 0;
  const double tf = 70.0;
  FiniteHorizonLinearQuadraticRegulatorOptions options;
  auto context = sys.CreateDefaultContext();
  const Eigen::Vector2d u0v = Eigen::Vector2d::Ones();
  sys.get_input_port().FixValue(context.get(), u0v);
  const Eigen::Vector2d udv = -B.inverse() * c;
  trajectories::PiecewisePolynomial<double> ud_traj(udv);
  options.ud = &ud_traj;
  options.N = N;
  options.Qf = lqr_result.S;

  FiniteHorizonLinearQuadraticRegulatorResult result =
      FiniteHorizonLinearQuadraticRegulator(sys, *context, t0, tf, Q, R,
                                            options);
  // The LQR x'Sx is the correct solution, because this is equivalent to solving
  // the linear system in xbar=(x-x0), ubar=(u-u0); and the LQR solution is
  // xbar'Sxbar, with x0=0.  However, in the finite-horizon version, all of the
  // affine terms must be correct to cancel each other out.
  EXPECT_TRUE(CompareMatrices(result.S->value(t0), lqr_result.S, 3.8e-5));
  EXPECT_TRUE(result.sx->value(t0).isZero(1e-5));
  EXPECT_TRUE(result.s0->value(t0).isZero(1e-5));
  // The LQR controller would be u = ud - Kx or u - u0 = -K(x-x0) - k0,
  // so Kx = lqr.K, k0 = -ud + u0.
  EXPECT_TRUE(CompareMatrices(result.K->value(t0), lqr_result.K, 4e-4));
  EXPECT_TRUE(CompareMatrices(result.k0->value(t0), -udv + u0v, 1e-5));

  // Test that the System version also works.
  const std::unique_ptr<System<double>> regulator =
      MakeFiniteHorizonLinearQuadraticRegulator(sys, *context, t0, tf, Q, R,
                                                options);
  auto regulator_context = regulator->CreateDefaultContext();
  const Eigen::Vector2d x(0.1, -0.3);
  regulator->get_input_port(0).FixValue(regulator_context.get(), x);
  EXPECT_TRUE(
      CompareMatrices(regulator->get_output_port(0).Eval(*regulator_context),
                      udv - lqr_result.K * x, 7.5e-5));
}

// Ensures that we can scalar convert the System version of the regulator.
GTEST_TEST(DiscreteTimeFiniteHorizonLQRTest, ResultSystemIsScalarConvertible) {
  const double h = 0.1;
  Eigen::Matrix2d A;
  Eigen::Vector2d B;
  A << 1, h, 0, 1;
  B << 0.5 * h * h, h;
  LinearSystem<double> sys(A, B, Eigen::Matrix<double, 0, 2>::Zero(),
                           Eigen::Matrix<double, 0, 1>::Zero(), h);
  auto context = sys.CreateDefaultContext();
  sys.get_input_port().FixValue(context.get(), 0.0);
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

  EXPECT_TRUE(is_symbolic_convertible(*regulator));
}

}  // namespace
}  // namespace controllers
}  // namespace systems
}  // namespace drake
