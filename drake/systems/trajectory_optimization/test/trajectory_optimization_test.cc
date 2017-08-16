#include <cmath>
#include <cstddef>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/systems/framework/vector_system.h"
#include "drake/systems/primitives/linear_system.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"
#include "drake/systems/trajectory_optimization/direct_trajectory_optimization.h"

using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

using drake::solvers::detail::VecIn;
using drake::solvers::detail::VecOut;

namespace drake {
namespace systems {
namespace {

typedef PiecewisePolynomial<double> PiecewisePolynomialType;

class MyDirectTrajOpt : public DirectTrajectoryOptimization {
 public:
  MyDirectTrajOpt(const int num_inputs, const int num_states,
                  const int num_time_samples, const double fixed_timestep)
      : DirectTrajectoryOptimization(num_inputs, num_states, num_time_samples,
                                     fixed_timestep) {}

  MyDirectTrajOpt(const int num_inputs, const int num_states,
                  const int num_time_samples, const double min_timestep,
                  const double max_timestep)
      : DirectTrajectoryOptimization(num_inputs, num_states, num_time_samples,
                                     min_timestep, max_timestep) {}

  // Expose for unit testing.
  using DirectTrajectoryOptimization::GetTimeVector;
  using DirectTrajectoryOptimization::h_vars;
  using DirectTrajectoryOptimization::timesteps_are_decision_variables;
  using DirectTrajectoryOptimization::fixed_timestep;

 private:
  void DoAddRunningCost(const symbolic::Expression& g) override {}
};

GTEST_TEST(TrajectoryOptimizationTest, FixedTimestepTest) {
  MyDirectTrajOpt prog(1, 2, 2,
                       0.1);  // 1 input, 2 states, 2 timesteps, 0.1<=dt<=0.1.

  EXPECT_FALSE(prog.timesteps_are_decision_variables());

  EXPECT_EQ(prog.num_vars(), 6);
  const auto times = prog.GetTimeVector();
  EXPECT_EQ(times.size(), 2);
  EXPECT_EQ(times[0], 0.0);
  EXPECT_EQ(times[1], 0.1);

  // All methods involving timestep variables should throw.
  EXPECT_THROW(prog.timestep(0), std::runtime_error);
  EXPECT_THROW(prog.fixed_timestep(), std::runtime_error);
  EXPECT_THROW(prog.AddTimeIntervalBounds(Eigen::Vector2d::Constant(0),
                                          Eigen::Vector2d::Constant(.1)),
               std::runtime_error);
  EXPECT_THROW(
      prog.AddTimeIntervalBounds(Eigen::Vector2d::Constant(0),
                                 Eigen::Vector2d::Constant(.1), {0, 1}),
      std::runtime_error);
  EXPECT_THROW(prog.AddTimeIntervalBounds(0, .1), std::runtime_error);
  EXPECT_THROW(prog.AddEqualTimeIntervalsConstraints(), std::runtime_error);
  EXPECT_THROW(prog.AddDurationBounds(0, 1), std::runtime_error);
}

GTEST_TEST(TrajectoryOptimizationTest, VariableTimestepTest) {
  MyDirectTrajOpt prog(1, 2, 2, 0.1,
                       0.1);  // 1 input, 2 states, 2 time samples, dt=0.1.

  EXPECT_TRUE(prog.timesteps_are_decision_variables());

  EXPECT_EQ(prog.num_vars(), 7);
  prog.Solve();
  const auto times = prog.GetTimeVector();
  EXPECT_EQ(times.size(), 2);
  EXPECT_EQ(times[0], 0.0);
  EXPECT_EQ(times[1], 0.1);
}

GTEST_TEST(TrajectoryOptimizationTest, PlaceholderVariableNames) {
  MyDirectTrajOpt prog(1, 2, 2,
                       0.1);  // 1 input, 2 states, 2 time samples, dt=0.1.

  EXPECT_EQ(prog.time().coeff(0).get_name(), "t");
  EXPECT_EQ(prog.state().coeff(0).get_name(), "x0");
  EXPECT_EQ(prog.state().coeff(1).get_name(), "x1");
  EXPECT_EQ(prog.input().coeff(0).get_name(), "u0");
}

GTEST_TEST(TrajectoryOptimizationTest, EqualTimeIntervalsTest) {
  MyDirectTrajOpt prog(1, 2, 3, 0.01,
                       0.3);  // 1 input, 2 states, 3 time samples (2 time
                              // steps), 0 <= dt <= 0.3.

  prog.AddEqualTimeIntervalsConstraints();

  prog.SetInitialGuess(prog.timestep(0), Vector1d(.1));
  prog.SetInitialGuess(prog.timestep(1), Vector1d(.2));

  prog.SetDecisionVariableValues(prog.initial_guess());
  EXPECT_NE(prog.GetSolution(prog.timestep(0)),
            prog.GetSolution(prog.timestep(1)));

  prog.Solve();
  EXPECT_EQ(prog.GetSolution(prog.timestep(0)),
            prog.GetSolution(prog.timestep(1)));
}

GTEST_TEST(TrajectoryOptimizationTest, DurationConstraintTest) {
  MyDirectTrajOpt prog(1, 2, 3, 0.01,
                       0.3);  // 1 input, 2 states, 3 time samples (2 time
                              // steps), 0 <= dt <= 0.3.

  prog.AddDurationBounds(.5, .5);

  prog.SetInitialGuess(prog.timestep(0), Vector1d(.1));
  prog.SetInitialGuess(prog.timestep(1), Vector1d(.2));

  prog.SetDecisionVariableValues(prog.initial_guess());
  EXPECT_NE(prog.GetSolution(prog.h_vars()).array().sum(), .5);

  prog.Solve();
  EXPECT_EQ(prog.GetSolution(prog.h_vars()).array().sum(), .5);
}

GTEST_TEST(TrajectoryOptimizationTest, InitialGuessTest) {
  MyDirectTrajOpt prog(1, 1, 3, 0.01, 0.5);

  const auto traj1 = PiecewisePolynomial<double>::FirstOrderHold(
      {1, 2}, {Vector1d(3), Vector1d(4)});
  const auto traj2 = PiecewisePolynomial<double>::FirstOrderHold(
      {1, 4}, {Vector1d(5), Vector1d(6)});

  // Throws if both are empty.
  EXPECT_THROW(prog.SetInitialTrajectory(PiecewisePolynomial<double>(),
                                         PiecewisePolynomial<double>()),
               std::runtime_error);

  // If one is empty, uses the duration.
  prog.SetInitialTrajectory(PiecewisePolynomial<double>(), traj1);
  prog.SetDecisionVariableValues(prog.initial_guess());
  EXPECT_EQ(prog.GetTimeVector(), std::vector<double>({0.0, 0.5, 1.0}));

  prog.SetInitialTrajectory(traj2, PiecewisePolynomial<double>());
  prog.SetDecisionVariableValues(prog.initial_guess());
  EXPECT_EQ(prog.GetTimeVector(), std::vector<double>({0.0, 1.5, 3.0}));

  // Throws if trajectories don't match.
  EXPECT_THROW(prog.SetInitialTrajectory(traj1, traj2), std::runtime_error);
}

GTEST_TEST(TrajectoryOptimizationTest, PlaceholderVariableTest) {
  const int kNumInputs(1);
  const int kNumStates(2);
  const int kNumTimeSamples(21);  // aka N.
  MyDirectTrajOpt prog(kNumInputs, kNumStates, kNumTimeSamples, 0.01, 25);

  // Adding a placeholder variable directly to the program should fail.
  auto u = prog.input();
  EXPECT_THROW(prog.AddCost(u(0)), std::runtime_error);
}

// qddot = u.
template <typename T>
class DoubleIntegrator : public VectorSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DoubleIntegrator);

  DoubleIntegrator()
      : VectorSystem<T>(SystemTypeTag<systems::DoubleIntegrator>{}, 1, 1) {
    this->DeclareContinuousState(1, 1, 0);
  }

  template <typename U>
  explicit DoubleIntegrator(const DoubleIntegrator<U>&)
      : DoubleIntegrator<T>() {}

  ~DoubleIntegrator() override {}

 protected:
  void DoCalcVectorOutput(
      const Context<T>& context,
      const Eigen::VectorBlock<const VectorX<T>>& input,
      const Eigen::VectorBlock<const VectorX<T>>& state,
      Eigen::VectorBlock<VectorX<T>>* output) const override {
    *output << state(0);
  }

  void DoCalcVectorTimeDerivatives(
      const Context<T>& context,
      const Eigen::VectorBlock<const VectorX<T>>& input,
      const Eigen::VectorBlock<const VectorX<T>>& state,
      Eigen::VectorBlock<VectorX<T>>* derivatives) const override {
    *derivatives << state(1), input(0);
  }
};

// Tests the double integrator minimum-time problem known solution.
GTEST_TEST(TrajectoryOptimizationTest, DoubleIntegratorTest) {
  DoubleIntegrator<double> double_integrator;
  auto context = double_integrator.CreateDefaultContext();
  const int timesteps{10};
  DircolTrajectoryOptimization prog(&double_integrator, *context, timesteps,
                                    0.5, 20.0);

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
  while (i < timesteps / 2.0)
    EXPECT_NEAR(prog.GetSolution(prog.input(i++))(0), 1.0, 1e-5);
  while (i < timesteps)
    EXPECT_NEAR(prog.GetSolution(prog.input(i++))(0), -1.0, 1e-5);
}

// Tests the double integrator without input limits results in minimal time.
GTEST_TEST(TrajectoryOptimizationTest, MinimumTimeTest) {
  DoubleIntegrator<double> double_integrator;
  auto context = double_integrator.CreateDefaultContext();
  const int timesteps{10};
  const double min_time{0.5};
  DircolTrajectoryOptimization prog(&double_integrator, *context, timesteps,
                                    min_time, 20.0);

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
  for (int i = 0; i < timesteps - 1; i++)
    total_time += prog.GetSolution(prog.timestep(i))(0);
  EXPECT_NEAR(total_time, min_time, 1e-5);
}

/// A simple example where the plant has no inputs.
GTEST_TEST(TrajectoryOptimizationTest, NoInputs) {
  // xdot = -x.
  systems::LinearSystem<double> plant(
      Vector1d(-1.0),                        // A
      Eigen::Matrix<double, 1, 0>::Zero(),   // B
      Eigen::Matrix<double, 0, 1>::Zero(),   // C
      Eigen::Matrix<double, 0, 0>::Zero());  // D

  auto context = plant.CreateDefaultContext();
  const int timesteps{10};
  const double duration{1.0};
  DircolTrajectoryOptimization prog(&plant, *context, timesteps, duration,
                                    duration);

  prog.AddTimeIntervalBounds(duration / (timesteps - 1),
                             duration / (timesteps - 1));

  const double x0 = 2.0;
  prog.AddLinearConstraint(prog.initial_state() == Vector1d(x0));

  EXPECT_EQ(prog.Solve(), solvers::SolutionResult::kSolutionFound);

  EXPECT_NEAR(prog.GetSolution(prog.final_state())(0), x0 * std::exp(-duration),
              1e-6);
}

}  // anonymous namespace
}  // namespace systems
}  // namespace drake
