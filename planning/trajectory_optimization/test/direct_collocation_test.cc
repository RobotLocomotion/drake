#include "drake/planning/trajectory_optimization/direct_collocation.h"

#include <cmath>
#include <cstddef>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/examples/rimless_wheel/rimless_wheel.h"
#include "drake/math/autodiff.h"
#include "drake/multibody/benchmarks/pendulum/make_pendulum_plant.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/solve.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace planning {
namespace trajectory_optimization {

using Eigen::VectorXd;
using solvers::Solve;
using systems::Context;
using systems::InputPortSelection;
using systems::LinearSystem;
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

std::unique_ptr<LinearSystem<double>> MakeDoubleIntegrator() {
  // Implements the double integrator: qddot = u.
  Eigen::Matrix2d A = Eigen::Matrix2d::Zero();
  A(0, 1) = 1.0;
  const Eigen::Vector2d B(0.0, 1.0);
  const Eigen::MatrixXd C(0, 2), D(0, 1);
  return std::make_unique<LinearSystem<double>>(A, B, C, D);
}

GTEST_TEST(DirectCollocationConstraint, DoubleConstructor) {
  const std::unique_ptr<LinearSystem<double>> system = MakeSimpleLinearSystem();
  const std::unique_ptr<Context<double>> context =
      system->CreateDefaultContext();

  // Make sure that the constraint can be constructed and evaluated.
  DirectCollocationConstraint constraint(*system, *context);
  const Eigen::VectorXd x = Eigen::VectorXd::Zero(constraint.num_vars());
  Eigen::VectorXd y = Eigen::VectorXd::Zero(constraint.num_constraints());
  constraint.Eval(x, &y);
}

GTEST_TEST(DirectCollocationConstraint, AutoDiffXdConstructor) {
  const std::unique_ptr<LinearSystem<double>> system = MakeSimpleLinearSystem();
  const std::unique_ptr<Context<double>> context =
      system->CreateDefaultContext();
  const auto system_ad = system->ToAutoDiffXd();
  auto context_ad = system_ad->CreateDefaultContext();

  // Make sure that the constraint can be constructed and evaluated.
  DirectCollocationConstraint constraint(*system_ad, context_ad.get(),
                                         context_ad.get(), context_ad.get());
  const Eigen::VectorXd x = Eigen::VectorXd::Zero(constraint.num_vars());
  Eigen::VectorXd y = Eigen::VectorXd::Zero(constraint.num_constraints());
  constraint.Eval(x, &y);
}

GTEST_TEST(DirectCollocation, PassProgToConstructor) {
  const std::unique_ptr<LinearSystem<double>> system = MakeSimpleLinearSystem();
  const std::unique_ptr<Context<double>> context =
      system->CreateDefaultContext();

  const int kNumSampleTimes = 4;
  const double kTimeStep = .1;
  solvers::MathematicalProgram prog;
  DirectCollocation dircol(
      system.get(), *context, kNumSampleTimes, kTimeStep, kTimeStep,
      systems::InputPortSelection::kUseFirstInputIfItExists, false, &prog);

  EXPECT_EQ(&prog, &dircol.prog());
  const int num_vars = prog.num_vars();

  // Add a second direct collocation problem to the same prog.
  DirectCollocation dircol2(
      system.get(), *context, kNumSampleTimes, kTimeStep, kTimeStep,
      systems::InputPortSelection::kUseFirstInputIfItExists, false, &prog);
  EXPECT_EQ(&prog, &dircol2.prog());
  EXPECT_EQ(prog.num_vars(), num_vars * 2);
}

GTEST_TEST(DirectCollocation, TestAddRunningCost) {
  const std::unique_ptr<LinearSystem<double>> system = MakeSimpleLinearSystem();
  const std::unique_ptr<Context<double>> context =
      system->CreateDefaultContext();

  const int kNumSampleTimes = 4;
  const double kTimeStep = .1;
  DirectCollocation dircol(system.get(), *context, kNumSampleTimes, kTimeStep,
                           kTimeStep);
  auto& prog = dircol.prog();

  dircol.AddRunningCost(
      dircol.state().cast<symbolic::Expression>().dot(dircol.state()) +
      dircol.input().cast<symbolic::Expression>().dot(dircol.input()));

  Eigen::Matrix<double, 2, kNumSampleTimes> u;
  Eigen::Matrix<double, 2, kNumSampleTimes> x;
  for (int i = 0; i < kNumSampleTimes - 1; ++i) {
    prog.SetInitialGuess(dircol.time_step(i), Vector1d(kTimeStep));
  }
  for (int i = 0; i < kNumSampleTimes; ++i) {
    x.col(i) << 0.2 * i - 1, 0.1 + i;
    u.col(i) << 0.1 * i, 0.2 * i + 0.1;
    prog.SetInitialGuess(dircol.state(i), x.col(i));
    prog.SetInitialGuess(dircol.input(i), u.col(i));
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
GTEST_TEST(DirectCollocation, TestCollocationConstraint) {
  const std::unique_ptr<LinearSystem<double>> system = MakeSimpleLinearSystem();
  const auto context = system->CreateDefaultContext();

  const int kNumSampleTimes = 3;
  const double kTimeStep = .1;
  DirectCollocation dircol(system.get(), *context, kNumSampleTimes, kTimeStep,
                           kTimeStep);
  auto& prog = dircol.prog();

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

    prog.SetInitialGuess(dircol.time_step(i), Vector1d(kTimeStep));
    prog.SetInitialGuess(dircol.input(i), u0);
    prog.SetInitialGuess(dircol.input(i + 1), u1);
    prog.SetInitialGuess(dircol.state(i), x0);
    prog.SetInitialGuess(dircol.state(i + 1), x1);

    EXPECT_TRUE(
        CompareMatrices(prog.EvalBindingAtInitialGuess(binding), defect, 1e-6));
  }
}

class EvaluationCounterSystem : public systems::LeafSystem<AutoDiffXd> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(EvaluationCounterSystem);

  EvaluationCounterSystem() {
    DeclareContinuousState(1);
    DeclareVectorInputPort("u", 1);
  }

  int evaluation_count() const { return evaluation_count_; }

 private:
  void DoCalcTimeDerivatives(
      const Context<AutoDiffXd>& context,
      systems::ContinuousState<AutoDiffXd>* derivatives) const override {
    evaluation_count_++;
    AutoDiffXd x = context.get_continuous_state_vector()[0];
    AutoDiffXd u = this->get_input_port().Eval(context)[0];
    (*derivatives)[0] = x + u;
  }

  mutable int evaluation_count_{0};
};

// Confirm that we can avoid evaluating the system derivatives using caching.
GTEST_TEST(DirectCollocation, TestCollocationConstraintCaching) {
  EvaluationCounterSystem system;
  const int kNumContexts = 3;
  std::vector<std::unique_ptr<Context<AutoDiffXd>>> sample_context(
      kNumContexts);
  for (int i = 0; i < kNumContexts; ++i) {
    sample_context[i] = system.CreateDefaultContext();
  }
  auto collocation_context = system.CreateDefaultContext();

  DirectCollocationConstraint constraint(system, sample_context[0].get(),
                                         sample_context[1].get(),
                                         collocation_context.get());

  VectorXd x(5);
  x << 0.1, 1.2, 2.3, 3.4, 4.56;
  VectorX<AutoDiffXd> x_ad = math::InitializeAutoDiff(x), y_ad(1);
  EXPECT_EQ(system.evaluation_count(), 0);
  constraint.Eval(x_ad, &y_ad);
  EXPECT_EQ(system.evaluation_count(), 3);
  // Same x, so this should not evaluate the dynamics.
  constraint.Eval(x_ad, &y_ad);
  EXPECT_EQ(system.evaluation_count(), 3);

  // A second constraint using shifted contexts
  DirectCollocationConstraint next_constraint(system, sample_context[1].get(),
                                              sample_context[2].get(),
                                              collocation_context.get());
  x << 0.1, x[2], 0.52, x[4], 0.25;
  x_ad = math::InitializeAutoDiff(x);
  // This should have a cache hit on sample_context[1], and so will evaluate
  // the dynamics 2 times instead of 3.
  next_constraint.Eval(x_ad, &y_ad);
  EXPECT_EQ(system.evaluation_count(), 5);

  // Changing the derivatives, but not the values, still hits the cache (zero
  // new dynamics evaluations).
  Eigen::MatrixXd derivatives(5, 2);
  derivatives << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10;
  x_ad = math::InitializeAutoDiff(x, derivatives);
  next_constraint.Eval(x_ad, &y_ad);
  EXPECT_EQ(system.evaluation_count(), 5);
}

// Checks the collocation constraint value against the interpolation used
// in the reconstructed trajectories.  This confirms that the reconstruction
// is using the same interpolation algorithms as the actual optimization.
GTEST_TEST(DirectCollocation, TestReconstruction) {
  const std::unique_ptr<LinearSystem<double>> system = MakeSimpleLinearSystem();
  const auto context = system->CreateDefaultContext();

  const int kNumSampleTimes = 3;
  const double kTimeStep = .1;
  DirectCollocation dircol(system.get(), *context, kNumSampleTimes, kTimeStep,
                           kTimeStep);
  auto& prog = dircol.prog();

  // Sets all decision variables to trivial known values (1,2,3,...).
  // Pretends that the solver has solved the optimization problem, and set the
  // decision variable to some user-specified values.
  solvers::MathematicalProgramResult result;
  result.set_decision_variable_index(prog.decision_variable_index());
  result.set_solver_id(solvers::SolverId("dummy"));
  result.set_x_val(
      Eigen::VectorXd::LinSpaced(prog.num_vars(), 1, prog.num_vars()));

  const PiecewisePolynomial<double> input_spline =
      dircol.ReconstructInputTrajectory(result);
  const PiecewisePolynomial<double> state_spline =
      dircol.ReconstructStateTrajectory(result);
  const auto derivative_spline = state_spline.derivative();

  double time = 0.0;
  for (int i = 0; i < kNumSampleTimes; i++) {
    EXPECT_TRUE(CompareMatrices(result.GetSolution(dircol.input(i)),
                                input_spline.value(time), 1e-6));
    EXPECT_TRUE(CompareMatrices(result.GetSolution(dircol.state(i)),
                                state_spline.value(time), 1e-6));

    EXPECT_TRUE(
        CompareMatrices(system->A() * result.GetSolution(dircol.state(i)) +
                            system->B() * result.GetSolution(dircol.input(i)),
                        derivative_spline.value(time), 1e-6));

    if (i < (kNumSampleTimes - 1)) {
      time += result.GetSolution(dircol.time_step(i).coeff(0));
    }
  }

  const std::vector<solvers::Binding<solvers::Constraint>>&
      collocation_constraints = prog.generic_constraints();
  EXPECT_EQ(collocation_constraints.size(), kNumSampleTimes - 1);
  time = 0.0;
  for (int i = 0; i < (kNumSampleTimes - 1); i++) {
    const double time_step = result.GetSolution(dircol.time_step(i).coeff(0));
    const double collocation_time = time + time_step / 2.0;

    const auto& binding = collocation_constraints[i];
    Eigen::Vector2d defect =
        derivative_spline.value(collocation_time) -
        system->A() * state_spline.value(collocation_time) -
        system->B() * input_spline.value(collocation_time);
    EXPECT_TRUE(CompareMatrices(
        defect, prog.EvalBinding(binding, result.get_x_val()), 1e-6));
    time += time_step;
  }
}

// Tests the double integrator minimum-time problem with the known solution.
GTEST_TEST(DirectCollocation, DoubleIntegratorTest) {
  const auto double_integrator = MakeDoubleIntegrator();
  auto context = double_integrator->CreateDefaultContext();
  const int kNumSampleTimes{10};
  DirectCollocation dircol(double_integrator.get(), *context, kNumSampleTimes,
                           0.05, 2.0);
  auto& prog = dircol.prog();

  dircol.AddEqualTimeIntervalsConstraints();

  // u \in [-1,1].
  dircol.AddConstraintToAllKnotPoints(Vector1d(-1.0) <= dircol.input());
  dircol.AddConstraintToAllKnotPoints(dircol.input() <= Vector1d(1.0));

  // xf = [0,0].
  prog.AddLinearConstraint(dircol.final_state().array() == 0.0);

  // x0 = [-1,0].
  prog.AddLinearConstraint(dircol.initial_state() ==
                           Eigen::Vector2d(-1.0, 0.0));

  // Cost is just total time.
  dircol.AddFinalCost(dircol.time().cast<symbolic::Expression>());

  const solvers::MathematicalProgramResult result = Solve(prog);
  EXPECT_TRUE(result.is_success());

  // Solution should be bang-band (u = +1 then -1).
  int i = 0;
  while (i < kNumSampleTimes / 2.0)
    EXPECT_NEAR(result.GetSolution(dircol.input(i++))(0), 1.0, 1e-5);
  while (i < kNumSampleTimes)
    EXPECT_NEAR(result.GetSolution(dircol.input(i++))(0), -1.0, 1e-5);
}

// Tests that the double integrator without input limits results in minimal
// time.
GTEST_TEST(DirectCollocation, MinimumTimeTest) {
  const auto double_integrator = MakeDoubleIntegrator();
  auto context = double_integrator->CreateDefaultContext();
  const int kNumSampleTimes{10};
  const double kMinTimeStep{0.05};
  const double kMaxTimeStep{2.0};
  DirectCollocation dircol(double_integrator.get(), *context, kNumSampleTimes,
                           kMinTimeStep, kMaxTimeStep);
  auto& prog = dircol.prog();

  dircol.AddEqualTimeIntervalsConstraints();

  // Note: No input limits this time.

  // xf = [0,0].
  prog.AddLinearConstraint(dircol.final_state().array() == 0.0);

  // x0 = [-1,0].
  prog.AddLinearConstraint(dircol.initial_state() ==
                           Eigen::Vector2d(-1.0, 0.0));

  // Cost is just total time.
  dircol.AddFinalCost(dircol.time().cast<symbolic::Expression>());

  const solvers::MathematicalProgramResult result = Solve(prog);
  EXPECT_TRUE(result.is_success());

  // Solution should have total time equal to 0.5.
  double total_time = 0;
  for (int i = 0; i < kNumSampleTimes - 1; i++)
    total_time += result.GetSolution(dircol.time_step(i))(0);
  EXPECT_NEAR(total_time, kMinTimeStep * (kNumSampleTimes - 1), 1e-5);
}

// A simple example where the plant has no inputs.
GTEST_TEST(DirectCollocation, NoInputs) {
  // xdot = -x.
  systems::LinearSystem<double> plant(
      Vector1d(-1.0),                        // A
      Eigen::Matrix<double, 1, 0>::Zero(),   // B
      Eigen::Matrix<double, 0, 1>::Zero(),   // C
      Eigen::Matrix<double, 0, 0>::Zero());  // D

  auto context = plant.CreateDefaultContext();
  const int kNumSampleTimes{10};
  const double kFixedTimeStep{0.1};
  DirectCollocation dircol(&plant, *context, kNumSampleTimes, kFixedTimeStep,
                           kFixedTimeStep);
  auto& prog = dircol.prog();

  const double x0 = 2.0;
  prog.AddLinearConstraint(dircol.initial_state() == Vector1d(x0));

  const solvers::MathematicalProgramResult result = Solve(prog);
  EXPECT_TRUE(result.is_success());

  const double duration = (kNumSampleTimes - 1) * kFixedTimeStep;
  EXPECT_NEAR(result.GetSolution(dircol.final_state())(0),
              x0 * std::exp(-duration), 1e-6);

  const auto state_trajectory = dircol.ReconstructStateTrajectory(result);
  EXPECT_EQ(state_trajectory.get_number_of_segments(), kNumSampleTimes - 1);
}

GTEST_TEST(DirectCollocation, AddDirectCollocationConstraint) {
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
  DirectCollocation dircol(plant.get(), *context, kNumSamples, kMinStep,
                           kMaxStep,
                           plant->get_actuation_input_port().get_index());
  auto& prog = dircol.prog();

  dircol.AddEqualTimeIntervalsConstraints();

  Eigen::Vector2d initial_state{0.0, 0.1};
  Eigen::Vector2d final_state{0.0, 0.0};

  prog.AddConstraint(dircol.initial_state() == initial_state);
  prog.AddConstraint(dircol.final_state() == final_state);

  const auto& u = dircol.input();
  dircol.AddRunningCost(10 * u[0] * u[0]);

  const auto result = Solve(prog);
  EXPECT_TRUE(result.is_success());
}

// Provide a helpful error if the user does *not* provide the
// InputPortSelection.
GTEST_TEST(DirectCollocation, InputPortSelectionError) {
  const auto plant = multibody::benchmarks::pendulum::MakePendulumPlant();
  auto context = plant->CreateDefaultContext();

  const int kNumSamples = 5;
  const double kMinStep = 0.05;
  const double kMaxStep = 0.5;
  DRAKE_EXPECT_THROWS_MESSAGE(
      DirectCollocation(plant.get(), *context, kNumSamples, kMinStep, kMaxStep),
      ".*non-default `input_port_index`.*");
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
  DirectCollocation dircol(&plant, *context, kNumSamples, kMinStep, kMaxStep,
                           InputPortSelection::kNoInput,
                           kAssumeNonContinuousStatesAreFixed);
  auto& prog = dircol.prog();

  const double slope = 0.08;
  const double alpha = M_PI / 8;  // half the interleg angle (in radians).
  dircol.AddEqualTimeIntervalsConstraints();
  dircol.AddConstraintToAllKnotPoints(dircol.state()[0] >= slope - alpha);
  dircol.AddConstraintToAllKnotPoints(dircol.state()[0] <= slope + alpha);

  prog.AddConstraint(dircol.initial_state()[0] == slope - alpha);
  prog.AddConstraint(dircol.final_state()[0] == slope + alpha);

  const auto result = Solve(prog);
  EXPECT_TRUE(result.is_success());
}

GTEST_TEST(DirectCollocation, DiscreteTimeSystemThrows) {
  const Vector1d one{1.0};
  const double time_period = 0.1;
  const LinearSystem<double> plant(one, one, one, one, time_period);
  auto context = plant.CreateDefaultContext();

  DRAKE_EXPECT_THROWS_MESSAGE(DirectCollocationConstraint(plant, *context),
                              ".*doesn't have any continuous states.*");

  const int kNumSamples = 15;
  const double kMinStep = 0.01;
  const double kMaxStep = 0.1;
  DRAKE_EXPECT_THROWS_MESSAGE(
      DirectCollocation(&plant, *context, kNumSamples, kMinStep, kMaxStep),
      ".*doesn't have any continuous states.*");
}

}  // namespace
}  // namespace trajectory_optimization
}  // namespace planning
}  // namespace drake
