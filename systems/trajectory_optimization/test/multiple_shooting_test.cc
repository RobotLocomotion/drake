#include "drake/systems/trajectory_optimization/multiple_shooting.h"

#include <cmath>
#include <cstddef>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {
namespace {

// TODO(russt): MathematicalProgram should provide this number for each solver.
const double kSolverTolerance = 1e-6;

typedef PiecewisePolynomial<double> PiecewisePolynomialType;

class MyDirectTrajOpt : public MultipleShooting {
 public:
  MyDirectTrajOpt(const int num_inputs, const int num_states,
                  const int num_time_samples, const double fixed_timestep)
      : MultipleShooting(num_inputs, num_states, num_time_samples,
                         fixed_timestep) {}

  MyDirectTrajOpt(const int num_inputs, const int num_states,
                  const int num_time_samples, const double min_timestep,
                  const double max_timestep)
      : MultipleShooting(num_inputs, num_states, num_time_samples, min_timestep,
                         max_timestep) {}

  PiecewisePolynomialTrajectory ReconstructInputTrajectory() const override {
    return PiecewisePolynomialTrajectory(PiecewisePolynomial<double>());
  };

  PiecewisePolynomialTrajectory ReconstructStateTrajectory() const override {
    return PiecewisePolynomialTrajectory(PiecewisePolynomial<double>());
  };

  // Expose for unit testing.
  using MultipleShooting::h_vars;
  using MultipleShooting::timesteps_are_decision_variables;
  using MultipleShooting::fixed_timestep;

 private:
  void DoAddRunningCost(const symbolic::Expression& g) override {}
};

GTEST_TEST(MultipleShootingTest, FixedTimestepTest) {
  const int kNumInputs{1};
  const int kNumStates{2};
  const int kNumSampleTimes{2};
  const double kFixedTimeStep{0.1};
  MyDirectTrajOpt prog(kNumInputs, kNumStates, kNumSampleTimes, kFixedTimeStep);

  EXPECT_FALSE(prog.timesteps_are_decision_variables());

  EXPECT_EQ(prog.num_vars(), 6);
  const Eigen::VectorXd times = prog.GetSampleTimes();
  EXPECT_EQ(times.size(), 2);
  EXPECT_EQ(times[0], 0.0);
  EXPECT_EQ(times[1], 0.1);

  // All methods involving timestep variables should throw.
  EXPECT_THROW(prog.timestep(0), std::runtime_error);
  EXPECT_THROW(prog.AddTimeIntervalBounds(0, .1), std::runtime_error);
  EXPECT_THROW(prog.AddEqualTimeIntervalsConstraints(), std::runtime_error);
  EXPECT_THROW(prog.AddDurationBounds(0, 1), std::runtime_error);
}

GTEST_TEST(MultipleShootingTest, VariableTimestepTest) {
  const int kNumInputs{1};
  const int kNumStates{2};
  const int kNumSampleTimes{2};
  const double kMinTimeStep{0.1};
  const double kMaxTimeStep{0.1};
  MyDirectTrajOpt prog(kNumInputs, kNumStates, kNumSampleTimes, kMinTimeStep,
                       kMaxTimeStep);

  EXPECT_TRUE(prog.timesteps_are_decision_variables());
  EXPECT_THROW(prog.fixed_timestep(), std::runtime_error);

  EXPECT_EQ(prog.num_vars(), 7);
  ASSERT_EQ(prog.Solve(), solvers::SolutionResult::kSolutionFound);
  const Eigen::VectorXd times = prog.GetSampleTimes();
  EXPECT_EQ(times.size(), 2);
  EXPECT_NEAR(times[0], 0.0, kSolverTolerance);
  EXPECT_NEAR(times[1], 0.1, kSolverTolerance);
}

GTEST_TEST(MultipleShootingTest, PlaceholderVariableTest) {
  const int kNumInputs{1};
  const int kNumStates{2};
  const int kNumSampleTimes{2};
  const double kMinTimeStep{0.01};
  const double kMaxTimeStep{1};
  MyDirectTrajOpt prog(kNumInputs, kNumStates, kNumSampleTimes, kMinTimeStep,
                       kMaxTimeStep);

  // Adding a placeholder variable directly to the program should fail.
  const solvers::VectorDecisionVariable<1>& t = prog.time();
  const solvers::VectorXDecisionVariable& u = prog.input();
  const solvers::VectorXDecisionVariable& x = prog.state();

  EXPECT_THROW(prog.AddCost(t(0)), std::runtime_error);
  EXPECT_THROW(prog.AddLinearCost(Vector1d(1.0), 0.0, u), std::runtime_error);
  EXPECT_THROW(prog.AddQuadraticErrorCost(Eigen::Matrix2d::Identity(),
                                          Eigen::Vector2d::Zero(), x),
               std::runtime_error);

  EXPECT_THROW(prog.AddLinearConstraint(t(0) <= 1.0), std::runtime_error);
  EXPECT_THROW(prog.AddLinearConstraint(u <= Vector1d(1.0)),
               std::runtime_error);

  EXPECT_THROW(prog.AddLinearConstraint(Eigen::Matrix2d::Identity(),
                                        Eigen::Vector2d::Zero(),
                                        Eigen::Vector2d::Zero(), x),
               std::runtime_error);

  EXPECT_THROW(prog.GetSolution(t(0)), std::runtime_error);
  EXPECT_THROW(prog.GetSolution(u), std::runtime_error);
}

GTEST_TEST(MultipleShootingTest, PlaceholderVariableNames) {
  const int kNumInputs{1};
  const int kNumStates{2};
  const int kNumSampleTimes{2};
  const double kFixedTimeStep{0.1};
  MyDirectTrajOpt prog(kNumInputs, kNumStates, kNumSampleTimes, kFixedTimeStep);

  EXPECT_EQ(prog.time().coeff(0).get_name(), "t");
  EXPECT_EQ(prog.state().coeff(0).get_name(), "x0");
  EXPECT_EQ(prog.state().coeff(1).get_name(), "x1");
  EXPECT_EQ(prog.input().coeff(0).get_name(), "u0");
}

GTEST_TEST(MultipleShootingTest, TimeIntervalBoundsTest) {
  const int kNumInputs{1};
  const int kNumStates{1};
  const int kNumSampleTimes{3};
  const double kMinTimeStep{0.01};
  const double kMaxTimeStep{1};
  MyDirectTrajOpt prog(kNumInputs, kNumStates, kNumSampleTimes, kMinTimeStep,
                       kMaxTimeStep);

  prog.AddTimeIntervalBounds(.5, .5);
  ASSERT_EQ(prog.Solve(), solvers::SolutionResult::kSolutionFound);
  EXPECT_TRUE(CompareMatrices(prog.GetSolution(prog.h_vars()),
                              Eigen::Vector2d(0.5, 0.5), 1e-6));
}

GTEST_TEST(MultipleShootingTest, EqualTimeIntervalsTest) {
  const int kNumInputs{1};
  const int kNumStates{2};
  const int kNumSampleTimes{3};
  const double kMinTimeStep{0.01};
  const double kMaxTimeStep{0.3};
  MyDirectTrajOpt prog(kNumInputs, kNumStates, kNumSampleTimes, kMinTimeStep,
                       kMaxTimeStep);

  prog.AddEqualTimeIntervalsConstraints();

  prog.SetInitialGuess(prog.timestep(0), Vector1d(.1));
  prog.SetInitialGuess(prog.timestep(1), Vector1d(.2));

  ASSERT_EQ(prog.Solve(), solvers::SolutionResult::kSolutionFound);
  EXPECT_NEAR(prog.GetSolution(prog.timestep(0).coeff(0)),
              prog.GetSolution(prog.timestep(1).coeff(0)), kSolverTolerance);
}

GTEST_TEST(MultipleShootingTest, DurationConstraintTest) {
  const int kNumInputs{0};
  const int kNumStates{1};
  const int kNumSampleTimes{3};
  const double kMinTimeStep{0.01};
  const double kMaxTimeStep{0.3};
  MyDirectTrajOpt prog(kNumInputs, kNumStates, kNumSampleTimes, kMinTimeStep,
                       kMaxTimeStep);

  prog.AddDurationBounds(.5, .5);

  prog.SetInitialGuess(prog.timestep(0), Vector1d(.1));
  prog.SetInitialGuess(prog.timestep(1), Vector1d(.2));

  // Note: IPOPT fails with unbounded variables.
  prog.AddConstraintToAllKnotPoints(prog.state() <= Vector1d(1));
  prog.AddConstraintToAllKnotPoints(prog.state() >= Vector1d(0));

  ASSERT_EQ(prog.Solve(), solvers::SolutionResult::kSolutionFound);
  EXPECT_NEAR(prog.GetSolution(prog.h_vars()).sum(), .5, 1e-6);
}

GTEST_TEST(MultipleShootingTest, ConstraintAllKnotsTest) {
  const int kNumInputs{1};
  const int kNumStates{2};
  const int kNumSampleTimes{3};
  const double kMinTimeStep{0.01};
  const double kMaxTimeStep{1};
  MyDirectTrajOpt prog(kNumInputs, kNumStates, kNumSampleTimes, kMinTimeStep,
                       kMaxTimeStep);

  const Eigen::Vector2d state_value(4.0, 5.0);
  prog.AddConstraintToAllKnotPoints(prog.state() == state_value);

  ASSERT_EQ(prog.Solve(), solvers::SolutionResult::kSolutionFound);
  for (int i = 0; i < kNumSampleTimes; i++) {
    EXPECT_TRUE(
        CompareMatrices(prog.GetSolution(prog.state(i)), state_value, 1e-6));
  }

  const solvers::VectorDecisionVariable<1>& t = prog.time();
  const solvers::VectorXDecisionVariable& u = prog.input();
  prog.AddConstraintToAllKnotPoints(u == t);
  ASSERT_EQ(prog.Solve(), solvers::SolutionResult::kSolutionFound);
  // u(0) = 0.
  EXPECT_NEAR(prog.GetSolution(prog.input(0).coeff(0)), 0.0, 1e-6);
  // u(1) = h(0).
  EXPECT_NEAR(prog.GetSolution(prog.input(1).coeff(0)),
              prog.GetSolution(prog.timestep(0).coeff(0)), 1e-6);
  // u(2) = h(0)+h(1).
  EXPECT_NEAR(prog.GetSolution(prog.input(2).coeff(0)),
              prog.GetSolution(prog.h_vars()).sum(), 1e-6);
}

GTEST_TEST(MultipleShootingTest, FinalCostTest) {
  const int kNumInputs{1};
  const int kNumStates{2};
  const int kNumSampleTimes{2};
  const double kFixedTimeStep{0.1};
  MyDirectTrajOpt prog(kNumInputs, kNumStates, kNumSampleTimes, kFixedTimeStep);

  // Set desired value for the final state.
  const Eigen::Vector2d desired_state(2.0, 3.0);
  const auto error = prog.state() - desired_state;

  prog.AddFinalCost(error.dot(error));
  ASSERT_EQ(prog.Solve(), solvers::SolutionResult::kSolutionFound);
  EXPECT_NEAR(prog.GetOptimalCost(), 0.0, kSolverTolerance);
  EXPECT_TRUE(CompareMatrices(prog.GetSolution(prog.state(1)), desired_state,
                              kSolverTolerance));
}

GTEST_TEST(MultipleShootingTest, InitialGuessTest) {
  const int kNumInputs{1};
  const int kNumStates{1};
  const int kNumSampleTimes{3};
  const double kMinTimeStep{0.01};
  const double kMaxTimeStep{0.5};
  MyDirectTrajOpt prog(kNumInputs, kNumStates, kNumSampleTimes, kMinTimeStep,
                       kMaxTimeStep);

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
  EXPECT_EQ(prog.GetSampleTimes(), Eigen::Vector3d(0.0, 0.5, 1.0));

  prog.SetInitialTrajectory(traj2, PiecewisePolynomial<double>());
  prog.SetDecisionVariableValues(prog.initial_guess());
  EXPECT_EQ(prog.GetSampleTimes(), Eigen::Vector3d(0.0, 1.5, 3.0));

  prog.SetInitialTrajectory(traj1, traj1);
  prog.SetDecisionVariableValues(prog.initial_guess());
  EXPECT_EQ(prog.GetSampleTimes(), Eigen::Vector3d(0.0, 0.5, 1.0));

  // Throws if trajectories don't match.
  EXPECT_THROW(prog.SetInitialTrajectory(traj1, traj2), std::runtime_error);
}

GTEST_TEST(MultipleShootingTest, ResultSamplesTest) {
  const int kNumInputs{1};
  const int kNumStates{2};
  const int kNumSampleTimes{2};
  const double kFixedTimeStep{0.1};
  MyDirectTrajOpt prog(kNumInputs, kNumStates, kNumSampleTimes, kFixedTimeStep);

  const Eigen::RowVector2d input_trajectory(1, 2);
  Eigen::Matrix2d state_trajectory;
  // clang-format off
  state_trajectory << 3, 4,
                      5, 6;
  // clang-format on

  for (int i = 0; i < kNumSampleTimes; i++) {
    prog.SetInitialGuess(prog.input(i), input_trajectory.col(i));
    prog.SetInitialGuess(prog.state(i), state_trajectory.col(i));
  }
  prog.SetDecisionVariableValues(prog.initial_guess());

  EXPECT_TRUE(CompareMatrices(prog.GetSampleTimes(),
                              Eigen::Vector2d(0.0, kFixedTimeStep), 0.0));
  EXPECT_TRUE(CompareMatrices(prog.GetInputSamples(), input_trajectory, 0.0));
  EXPECT_TRUE(CompareMatrices(prog.GetStateSamples(), state_trajectory, 0.0));
}

}  // anonymous namespace
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
