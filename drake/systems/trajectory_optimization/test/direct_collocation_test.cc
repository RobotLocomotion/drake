#include "drake/systems/trajectory_optimization/direct_collocation.h"

#include <cmath>
#include <cstddef>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {
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
  const auto segment = PiecewisePolynomial<double>::Cubic(
      {0.0, kTimeStep}, {x0, x1}, xdot0, xdot1);
  const auto derivative = segment.derivative();
  const Eigen::Vector2d defect = derivative.value(kTimeStep / 2.0) -
                                 system->A() * segment.value(kTimeStep / 2.0) -
                                 system->B() * (u0 + u1) / 2.0;

  for (int i = 0; i < (kNumSampleTimes - 1); i++) {
    const auto& binding = collocation_constraints[i];

    prog.SetDecisionVariableValues(prog.timestep(i), Vector1d(kTimeStep));
    prog.SetDecisionVariableValues(prog.input(i), u0);
    prog.SetDecisionVariableValues(prog.input(i + 1), u1);
    prog.SetDecisionVariableValues(prog.state(i), x0);
    prog.SetDecisionVariableValues(prog.state(i + 1), x1);

    EXPECT_TRUE(
        CompareMatrices(prog.EvalBindingAtSolution(binding), defect, 1e-6));
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
  prog.SetDecisionVariableValues(
      Eigen::VectorXd::LinSpaced(prog.num_vars(), 1, prog.num_vars()));

  const PiecewisePolynomialTrajectory input_spline =
      prog.ReconstructInputTrajectory();
  const PiecewisePolynomialTrajectory state_spline =
      prog.ReconstructStateTrajectory();
  const auto derivative_spline = state_spline.derivative();

  double time = 0.0;
  for (int i = 0; i < kNumSampleTimes; i++) {
    EXPECT_TRUE(CompareMatrices(prog.GetSolution(prog.input(i)),
                                input_spline.value(time), 1e-6));
    EXPECT_TRUE(CompareMatrices(prog.GetSolution(prog.state(i)),
                                state_spline.value(time), 1e-6));

    EXPECT_TRUE(
        CompareMatrices(system->A() * prog.GetSolution(prog.state(i)) +
                            system->B() * prog.GetSolution(prog.input(i)),
                        derivative_spline->value(time), 1e-6));

    if (i < (kNumSampleTimes - 1)) {
      time += prog.GetSolution(prog.timestep(i).coeff(0));
    }
  }

  const std::vector<solvers::Binding<solvers::Constraint>>&
      collocation_constraints = prog.generic_constraints();
  EXPECT_EQ(collocation_constraints.size(), kNumSampleTimes - 1);
  time = 0.0;
  for (int i = 0; i < (kNumSampleTimes - 1); i++) {
    const double timestep = prog.GetSolution(prog.timestep(i).coeff(0));
    const double collocation_time = time + timestep / 2.0;

    const auto& binding = collocation_constraints[i];
    Eigen::Vector2d defect =
        derivative_spline->value(collocation_time) -
        system->A() * state_spline.value(collocation_time) -
        system->B() * input_spline.value(collocation_time);
    EXPECT_TRUE(
        CompareMatrices(defect, prog.EvalBindingAtSolution(binding), 1e-6));
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

  EXPECT_EQ(prog.Solve(), solvers::SolutionResult::kSolutionFound);

  // Solution should be bang-band (u = +1 then -1).
  int i = 0;
  while (i < kNumSampleTimes / 2.0)
    EXPECT_NEAR(prog.GetSolution(prog.input(i++))(0), 1.0, 1e-5);
  while (i < kNumSampleTimes)
    EXPECT_NEAR(prog.GetSolution(prog.input(i++))(0), -1.0, 1e-5);
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

  EXPECT_EQ(prog.Solve(), solvers::SolutionResult::kSolutionFound);

  // Solution should have total time equal to 0.5.
  double total_time = 0;
  for (int i = 0; i < kNumSampleTimes - 1; i++)
    total_time += prog.GetSolution(prog.timestep(i))(0);
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

  EXPECT_EQ(prog.Solve(), solvers::SolutionResult::kSolutionFound);

  const double duration = (kNumSampleTimes - 1) * kFixedTimeStep;
  EXPECT_NEAR(prog.GetSolution(prog.final_state())(0), x0 * std::exp(-duration),
              1e-6);
}

}  // anonymous namespace
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
