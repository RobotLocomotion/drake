#include "drake/systems/trajectory_optimization/direct_collocation.h"

#include <cmath>
#include <cstddef>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/examples/rimless_wheel/rimless_wheel.h"
#include "drake/math/autodiff.h"
#include "drake/multibody/benchmarks/pendulum/make_pendulum_plant.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {

using trajectories::PiecewisePolynomial;

namespace {

std::unique_ptr<LinearSystem<double>> MakeSimpleLinearSystem() {
  Eigen::Matrix2d A, B;
  // clang-format off
  A << 1, 2,
       3, 4;
  B << 5, 6,
       7, 8;
  // clang-format on
  const Eigen::MatrixXd C(0, 2), D(0, 2);
  return std::make_unique<LinearSystem<double>>(A, B, C, D);
}

GTEST_TEST(DirectCollocationTest, TestAddRunningCost) {
  const std::unique_ptr<LinearSystem<double>> system = MakeSimpleLinearSystem();
  const std::unique_ptr<Context<double>> context =
      system->CreateDefaultContext();

  const int kNumSampleTimes = 4;
  const double kTimeStep = .1;
  DirectCollocation prog(system.get(), *context, kNumSampleTimes, kTimeStep,
                         kTimeStep);

  prog.AddRunningCost(
      prog.state().cast<symbolic::Expression>().dot(prog.state()) +
      prog.input().cast<symbolic::Expression>().dot(prog.input()));

  Eigen::Matrix<double, 2, kNumSampleTimes> u;
  Eigen::Matrix<double, 2, kNumSampleTimes> x;
  for (int i = 0; i < kNumSampleTimes - 1; ++i) {
    prog.SetInitialGuess(prog.timestep(i), Vector1d(kTimeStep));
  }
  for (int i = 0; i < kNumSampleTimes; ++i) {
    x.col(i) << 0.2 * i - 1, 0.1 + i;
    u.col(i) << 0.1 * i, 0.2 * i + 0.1;
    prog.SetInitialGuess(prog.state(i), x.col(i));
    prog.SetInitialGuess(prog.input(i), u.col(i));
  }
  double total_cost = 0;
  for (const auto& cost : prog.GetAllCosts()) {
    total_cost += prog.EvalBindingAtInitialGuess(cost)(0);
  }
  const Eigen::Matrix<double, 1, kNumSampleTimes> g_val =
      (x.array() * x.array()).matrix().colwise().sum() +
      (u.array() * u.array()).matrix().colwise().sum();
  const double total_cost_expected =
      ((g_val.head<kNumSampleTimes - 1>() + g_val.tail<kNumSampleTimes - 1>()) /
       2 * kTimeStep)
          .sum();
  EXPECT_NEAR(total_cost, total_cost_expected, 1E-12);
}

// Reconstructs the collocation constraint value (called the "defect") from a
// completely separate code path (using the PiecewisePolynomial methods).
GTEST_TEST(DirectCollocationTest, TestCollocationConstraint) {
  const std::unique_ptr<LinearSystem<double>> system = MakeSimpleLinearSystem();
  const auto context = system->CreateDefaultContext();

  const int kNumSampleTimes = 3;
  const double kTimeStep = .1;
  DirectCollocation prog(system.get(), *context, kNumSampleTimes, kTimeStep,
                         kTimeStep);

  // Constructor should add collocation constraints, and these are the
  // only generic constraints that should be in the program so far.
  const std::vector<solvers::Binding<solvers::Constraint>>&
      collocation_constraints = prog.generic_constraints();
  EXPECT_EQ(collocation_constraints.size(), kNumSampleTimes - 1);

  // Set up the cubic spline that should represent the state trajectory at any
  // given segment.
  const Eigen::Vector2d x0(6, 7), x1(8, 9), u0(10, 11), u1(12, 13);
  const Eigen::Vector2d xdot0 = system->A() * x0 + system->B() * u0;
  const Eigen::Vector2d xdot1 = system->A() * x1 + system->B() * u1;
  const auto segment =
      PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
          {0.0, kTimeStep}, {x0, x1}, xdot0, xdot1);
  const auto derivative = segment.derivative();
  const Eigen::Vector2d defect = derivative.value(kTimeStep / 2.0) -
                                 system->A() * segment.value(kTimeStep / 2.0) -
                                 system->B() * (u0 + u1) / 2.0;

  for (int i = 0; i < (kNumSampleTimes - 1); i++) {
    const auto& binding = collocation_constraints[i];

    prog.SetInitialGuess(prog.timestep(i), Vector1d(kTimeStep));
    prog.SetInitialGuess(prog.input(i), u0);
    prog.SetInitialGuess(prog.input(i + 1), u1);
    prog.SetInitialGuess(prog.state(i), x0);
    prog.SetInitialGuess(prog.state(i + 1), x1);

    EXPECT_TRUE(
        CompareMatrices(prog.EvalBindingAtInitialGuess(binding), defect, 1e-6));
  }
}

// Checks the collocation constraint value against the interpolation used
// in the reconstructed trajectories.  This confirms that the reconstruction
// is using the same interpolation algorithms as the actual optimization.
GTEST_TEST(DirectCollocationTest, TestReconstruction) {
  const std::unique_ptr<LinearSystem<double>> system = MakeSimpleLinearSystem();
  const auto context = system->CreateDefaultContext();

  const int kNumSampleTimes = 3;
  const double kTimeStep = .1;
  DirectCollocation prog(system.get(), *context, kNumSampleTimes, kTimeStep,
                         kTimeStep);

  // Sets all decision variables to trivial known values (1,2,3,...).
  // Pretends that the solver has solved the optimization problem, and set the
  // decision variable to some user-specified values.
  solvers::MathematicalProgramResult result;
  result.set_decision_variable_index(prog.decision_variable_index());
  result.set_solver_id(solvers::SolverId("dummy"));
  result.set_x_val(
      Eigen::VectorXd::LinSpaced(prog.num_vars(), 1, prog.num_vars()));

  const PiecewisePolynomial<double> input_spline =
      prog.ReconstructInputTrajectory(result);
  const PiecewisePolynomial<double> state_spline =
      prog.ReconstructStateTrajectory(result);
  const auto derivative_spline = state_spline.derivative();

  double time = 0.0;
  for (int i = 0; i < kNumSampleTimes; i++) {
    EXPECT_TRUE(CompareMatrices(result.GetSolution(prog.input(i)),
                                input_spline.value(time), 1e-6));
    EXPECT_TRUE(CompareMatrices(result.GetSolution(prog.state(i)),
                                state_spline.value(time), 1e-6));

    EXPECT_TRUE(
        CompareMatrices(system->A() * result.GetSolution(prog.state(i)) +
                            system->B() * result.GetSolution(prog.input(i)),
                        derivative_spline.value(time), 1e-6));

    if (i < (kNumSampleTimes - 1)) {
      time += result.GetSolution(prog.timestep(i).coeff(0));
    }
  }

  const std::vector<solvers::Binding<solvers::Constraint>>&
      collocation_constraints = prog.generic_constraints();
  EXPECT_EQ(collocation_constraints.size(), kNumSampleTimes - 1);
  time = 0.0;
  for (int i = 0; i < (kNumSampleTimes - 1); i++) {
    const double timestep = result.GetSolution(prog.timestep(i).coeff(0));
    const double collocation_time = time + timestep / 2.0;

    const auto& binding = collocation_constraints[i];
    Eigen::Vector2d defect =
        derivative_spline.value(collocation_time) -
        system->A() * state_spline.value(collocation_time) -
        system->B() * input_spline.value(collocation_time);
    EXPECT_TRUE(CompareMatrices(
        defect, prog.EvalBinding(binding, result.get_x_val()), 1e-6));
    time += timestep;
  }
}

std::unique_ptr<LinearSystem<double>> MakeDoubleIntegrator() {
  // Implements the double integrator: qddot = u.
  Eigen::Matrix2d A = Eigen::Matrix2d::Zero();
  A(0, 1) = 1.0;
  const Eigen::Vector2d B(0.0, 1.0);
  const Eigen::MatrixXd C(0, 2), D(0, 1);
  return std::make_unique<LinearSystem<double>>(A, B, C, D);
}

// Tests the double integrator minimum-time problem with the known solution.
GTEST_TEST(DirectCollocationTest, DoubleIntegratorTest) {
  const auto double_integrator = MakeDoubleIntegrator();
  auto context = double_integrator->CreateDefaultContext();
  const int kNumSampleTimes{10};
  DirectCollocation prog(double_integrator.get(), *context, kNumSampleTimes,
                         0.05, 2.0);

  prog.AddEqualTimeIntervalsConstraints();

  // u \in [-1,1].
  prog.AddConstraintToAllKnotPoints(Vector1d(-1.0) <= prog.input());
  prog.AddConstraintToAllKnotPoints(prog.input() <= Vector1d(1.0));

  // xf = [0,0].
  prog.AddLinearConstraint(prog.final_state().array() == 0.0);

  // x0 = [-1,0].
  prog.AddLinearConstraint(prog.initial_state() == Eigen::Vector2d(-1.0, 0.0));

  // Cost is just total time.
  prog.AddFinalCost(prog.time().cast<symbolic::Expression>());

  const solvers::MathematicalProgramResult result = Solve(prog);
  EXPECT_TRUE(result.is_success());

  // Solution should be bang-band (u = +1 then -1).
  int i = 0;
  while (i < kNumSampleTimes / 2.0)
    EXPECT_NEAR(result.GetSolution(prog.input(i++))(0), 1.0, 1e-5);
  while (i < kNumSampleTimes)
    EXPECT_NEAR(result.GetSolution(prog.input(i++))(0), -1.0, 1e-5);
}

// Tests that the double integrator without input limits results in minimal
// time.
GTEST_TEST(DirectCollocationTest, MinimumTimeTest) {
  const auto double_integrator = MakeDoubleIntegrator();
  auto context = double_integrator->CreateDefaultContext();
  const int kNumSampleTimes{10};
  const double kMinTimeStep{0.05};
  const double kMaxTimeStep{2.0};
  DirectCollocation prog(double_integrator.get(), *context, kNumSampleTimes,
                         kMinTimeStep, kMaxTimeStep);

  prog.AddEqualTimeIntervalsConstraints();

  // Note: No input limits this time.

  // xf = [0,0].
  prog.AddLinearConstraint(prog.final_state().array() == 0.0);

  // x0 = [-1,0].
  prog.AddLinearConstraint(prog.initial_state() == Eigen::Vector2d(-1.0, 0.0));

  // Cost is just total time.
  prog.AddFinalCost(prog.time().cast<symbolic::Expression>());

  const solvers::MathematicalProgramResult result = Solve(prog);
  EXPECT_TRUE(result.is_success());

  // Solution should have total time equal to 0.5.
  double total_time = 0;
  for (int i = 0; i < kNumSampleTimes - 1; i++)
    total_time += result.GetSolution(prog.timestep(i))(0);
  EXPECT_NEAR(total_time, kMinTimeStep * (kNumSampleTimes - 1), 1e-5);
}

// A simple example where the plant has no inputs.
GTEST_TEST(DirectCollocationTest, NoInputs) {
  // xdot = -x.
  systems::LinearSystem<double> plant(
      Vector1d(-1.0),                        // A
      Eigen::Matrix<double, 1, 0>::Zero(),   // B
      Eigen::Matrix<double, 0, 1>::Zero(),   // C
      Eigen::Matrix<double, 0, 0>::Zero());  // D

  auto context = plant.CreateDefaultContext();
  const int kNumSampleTimes{10};
  const double kFixedTimeStep{0.1};
  DirectCollocation prog(&plant, *context, kNumSampleTimes, kFixedTimeStep,
                         kFixedTimeStep);

  const double x0 = 2.0;
  prog.AddLinearConstraint(prog.initial_state() == Vector1d(x0));

  const solvers::MathematicalProgramResult result = Solve(prog);
  EXPECT_TRUE(result.is_success());

  const double duration = (kNumSampleTimes - 1) * kFixedTimeStep;
  EXPECT_NEAR(result.GetSolution(prog.final_state())(0),
              x0 * std::exp(-duration), 1e-6);

  const auto state_trajectory = prog.ReconstructStateTrajectory(result);
  EXPECT_EQ(state_trajectory.get_number_of_segments(), kNumSampleTimes - 1);
}

GTEST_TEST(DirectCollocationTest, AddDirectCollocationConstraint) {
  const auto double_integrator = MakeDoubleIntegrator();
  auto context = double_integrator->CreateDefaultContext();
  auto constraint = std::make_shared<DirectCollocationConstraint>(
      *double_integrator, *context);

  solvers::MathematicalProgram prog;
  const auto h = prog.NewContinuousVariables<1>();
  const auto x0 = prog.NewContinuousVariables<2>();
  const auto x1 = prog.NewContinuousVariables<2>();
  const auto u0 = prog.NewContinuousVariables<1>();
  const auto u1 = prog.NewContinuousVariables<1>();

  solvers::Binding<solvers::Constraint> binding =
      AddDirectCollocationConstraint(constraint, h, x0, x1, u0, u1, &prog);

  EXPECT_EQ(prog.generic_constraints().size(), 1);

  // qdot = 0, u = 0 should be a fixed point for any q.  Test a simple one.
  prog.SetInitialGuess(h, Vector1d{1.0});
  prog.SetInitialGuess(x0, Eigen::Vector2d{1., 0.});
  prog.SetInitialGuess(x1, Eigen::Vector2d{1., 0.});
  prog.SetInitialGuess(u0, Vector1d{0.});
  prog.SetInitialGuess(u1, Vector1d{0.});

  const Eigen::VectorXd val = prog.EvalBindingAtInitialGuess(binding);
  EXPECT_EQ(val.size(), 2);
  EXPECT_TRUE(val.isZero());
}

// Almost any optimization with MultibodyPlant will need the input port
// selection feature.  Add a simple example here.
GTEST_TEST(DirectCollocation, InputPortSelection) {
  const auto plant = multibody::benchmarks::pendulum::MakePendulumPlant();
  auto context = plant->CreateDefaultContext();

  const int kNumSamples = 5;
  const double kMinStep = 0.05;
  const double kMaxStep = 0.5;
  DirectCollocation prog(plant.get(), *context, kNumSamples, kMinStep, kMaxStep,
                         plant->get_actuation_input_port().get_index());

  prog.AddEqualTimeIntervalsConstraints();

  Eigen::Vector2d initial_state{0.0, 0.1};
  Eigen::Vector2d final_state{0.0, 0.0};

  prog.AddConstraint(prog.initial_state() == initial_state);
  prog.AddConstraint(prog.final_state() == final_state);

  const auto& u = prog.input();
  prog.AddRunningCost(10 * u[0] * u[0]);

  const auto result = Solve(prog);
  EXPECT_TRUE(result.is_success());
}

// The Rimless Wheel example has discrete state for book-keeping only.  The
// following is a simple example of effectively simulating one "step" of the
// wheel using direct collocation; this system was a motivating example for
// allowing the target system to have non-participating discrete/abstract
// states.
GTEST_TEST(DirectCollocation, IgnoreNonContinuousState) {
  examples::rimless_wheel::RimlessWheel<double> plant;
  auto context = plant.CreateDefaultContext();

  const int kNumSamples = 15;
  const double kMinStep = 0.01;
  const double kMaxStep = 0.1;
  const bool kAssumeNonContinuousStatesAreFixed = true;
  DirectCollocation prog(&plant, *context, kNumSamples, kMinStep, kMaxStep,
                         InputPortSelection::kNoInput,
                         kAssumeNonContinuousStatesAreFixed);

  const double slope = 0.08;
  const double alpha = M_PI / 8;  // half the interleg angle (in radians).
  prog.AddEqualTimeIntervalsConstraints();
  prog.AddConstraintToAllKnotPoints(prog.state()[0] >= slope - alpha);
  prog.AddConstraintToAllKnotPoints(prog.state()[0] <= slope + alpha);

  prog.AddConstraint(prog.initial_state()[0] == slope - alpha);
  prog.AddConstraint(prog.final_state()[0] == slope + alpha);

  const auto result = Solve(prog);
  EXPECT_TRUE(result.is_success());
}

}  // anonymous namespace
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
