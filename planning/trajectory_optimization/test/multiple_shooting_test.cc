#include "drake/planning/trajectory_optimization/multiple_shooting.h"

#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace planning {
namespace trajectory_optimization {
namespace {

using solvers::Solve;

// TODO(russt): MathematicalProgram should provide this number for each solver.
const double kSolverTolerance = 1e-6;

using trajectories::PiecewisePolynomial;
typedef PiecewisePolynomial<double> PiecewisePolynomialType;

class MyDirectTrajOpt : public MultipleShooting {
 public:
  MyDirectTrajOpt(const int num_inputs, const int num_states,
                  const int num_time_samples, const double fixed_time_step,
                  solvers::MathematicalProgram* prog = nullptr)
      : MultipleShooting(num_inputs, num_states, num_time_samples,
                         fixed_time_step, prog) {}

  MyDirectTrajOpt(const int num_inputs, const int num_states,
                  const int num_time_samples, const double min_time_step,
                  const double max_time_step,
                  solvers::MathematicalProgram* prog = nullptr)
      : MultipleShooting(num_inputs, num_states, num_time_samples,
                         min_time_step, max_time_step, prog) {}

  MyDirectTrajOpt(const solvers::VectorXDecisionVariable& input,
                  const solvers::VectorXDecisionVariable& state,
                  const int num_time_samples, const double fixed_time_step,
                  solvers::MathematicalProgram* prog = nullptr)
      : MultipleShooting(input, state, num_time_samples, fixed_time_step,
                         prog) {}

  MyDirectTrajOpt(const solvers::VectorXDecisionVariable& input,
                  const solvers::VectorXDecisionVariable& state,
                  const symbolic::Variable& time, const int num_time_samples,
                  const double min_time_step, const double max_time_step,
                  solvers::MathematicalProgram* prog = nullptr)
      : MultipleShooting(input, state, time, num_time_samples, min_time_step,
                         max_time_step, prog) {}

  PiecewisePolynomial<double> ReconstructInputTrajectory(
      const solvers::MathematicalProgramResult&) const override {
    return PiecewisePolynomial<double>();
  };

  PiecewisePolynomial<double> ReconstructStateTrajectory(
      const solvers::MathematicalProgramResult&) const override {
    return PiecewisePolynomial<double>();
  };

  // Expose for unit testing.
  using MultipleShooting::fixed_time_step;
  using MultipleShooting::GetSequentialVariable;
  using MultipleShooting::h_vars;
  using MultipleShooting::time_steps_are_decision_variables;
  using MultipleShooting::u_vars;
  using MultipleShooting::x_vars;

 private:
  void DoAddRunningCost(const symbolic::Expression& g) override {}
};

GTEST_TEST(MultipleShootingTest, FixedTimestepTest) {
  const int kNumInputs{1};
  const int kNumStates{2};
  const int kNumSampleTimes{2};
  const double kFixedTimeStep{0.1};
  MyDirectTrajOpt trajopt(kNumInputs, kNumStates, kNumSampleTimes,
                          kFixedTimeStep);

  EXPECT_FALSE(trajopt.time_steps_are_decision_variables());

  EXPECT_EQ(trajopt.prog().num_vars(), 6);
  solvers::MathematicalProgramResult result;
  const Eigen::VectorXd times = trajopt.GetSampleTimes(result);
  EXPECT_EQ(times.size(), 2);
  EXPECT_EQ(times[0], 0.0);
  EXPECT_EQ(times[1], 0.1);

  // All methods involving time_step variables should throw.
  EXPECT_THROW(trajopt.time_step(0), std::runtime_error);
  EXPECT_THROW(trajopt.AddTimeIntervalBounds(0, .1), std::runtime_error);
  EXPECT_THROW(trajopt.AddEqualTimeIntervalsConstraints(), std::runtime_error);
  EXPECT_THROW(trajopt.AddDurationBounds(0, 1), std::runtime_error);
}

GTEST_TEST(MultipleShootingTest, FixedTimestepWithProgTest) {
  const int kNumInputs{1};
  const int kNumStates{2};
  const int kNumSampleTimes{2};
  const double kFixedTimeStep{0.1};
  solvers::MathematicalProgram prog;

  MyDirectTrajOpt trajopt(kNumInputs, kNumStates, kNumSampleTimes,
                          kFixedTimeStep, &prog);
  EXPECT_EQ(&prog, &trajopt.prog());
  EXPECT_EQ(prog.num_vars(), 6);

  MyDirectTrajOpt trajopt2(kNumInputs, kNumStates, kNumSampleTimes,
                           kFixedTimeStep, &prog);
  EXPECT_EQ(&prog, &trajopt2.prog());
  EXPECT_EQ(prog.num_vars(), 12);
}

GTEST_TEST(MultipleShootingTest, FixedTimestepTestWithPlaceholderVariables) {
  const VectorX<symbolic::Variable> input{
      symbolic::MakeVectorContinuousVariable(3, "my_input")};
  const VectorX<symbolic::Variable> state{
      symbolic::MakeVectorContinuousVariable(4, "my_state")};
  const int kNumSampleTimes{2};
  const double kFixedTimeStep{0.1};
  MyDirectTrajOpt trajopt(input, state, kNumSampleTimes, kFixedTimeStep);

  EXPECT_FALSE(trajopt.time_steps_are_decision_variables());

  EXPECT_EQ(trajopt.prog().num_vars(),
            (input.size() + state.size()) * kNumSampleTimes);
  solvers::MathematicalProgramResult result;
  const Eigen::VectorXd times = trajopt.GetSampleTimes(result);
  EXPECT_EQ(times.size(), 2);
  EXPECT_EQ(times[0], 0.0);
  EXPECT_EQ(times[1], 0.1);

  const solvers::VectorXDecisionVariable& u = trajopt.input();
  EXPECT_EQ(u, input);

  const solvers::VectorXDecisionVariable& x = trajopt.state();
  EXPECT_EQ(x, state);

  // All methods involving time step variables should throw.
  EXPECT_THROW(trajopt.time_step(0), std::runtime_error);
  EXPECT_THROW(trajopt.AddTimeIntervalBounds(0, .1), std::runtime_error);
  EXPECT_THROW(trajopt.AddEqualTimeIntervalsConstraints(), std::runtime_error);
  EXPECT_THROW(trajopt.AddDurationBounds(0, 1), std::runtime_error);

  // Now use the constructor which takes a prog.
  MyDirectTrajOpt trajopt2(input, state, kNumSampleTimes, kFixedTimeStep,
                           &trajopt.prog());
  EXPECT_EQ(trajopt.prog().num_vars(),
            2 * (input.size() + state.size()) * kNumSampleTimes);
}

GTEST_TEST(MultipleShootingTest, VariableTimestepTestWithPlaceholderVariables) {
  const VectorX<symbolic::Variable> input{
      symbolic::MakeVectorContinuousVariable(3, "my_input")};
  const VectorX<symbolic::Variable> state{
      symbolic::MakeVectorContinuousVariable(4, "my_state")};
  const symbolic::Variable time{"my_time"};
  const int kNumSampleTimes{2};
  const double kMinTimeStep{0.1};
  const double kMaxTimeStep{0.1};
  MyDirectTrajOpt trajopt(input, state, time, kNumSampleTimes, kMinTimeStep,
                          kMaxTimeStep);

  const solvers::VectorXDecisionVariable& u = trajopt.input();
  EXPECT_EQ(u, input);

  const solvers::VectorXDecisionVariable& x = trajopt.state();
  EXPECT_EQ(x, state);

  EXPECT_EQ(time, trajopt.time()[0]);

  EXPECT_TRUE(trajopt.time_steps_are_decision_variables());
  EXPECT_THROW(trajopt.fixed_time_step(), std::runtime_error);

  EXPECT_EQ(
      trajopt.prog().num_vars(),
      (input.size() + state.size()) * kNumSampleTimes + 1 /* time variable */);
  const solvers::MathematicalProgramResult result = Solve(trajopt.prog());
  ASSERT_TRUE(result.is_success());
  const Eigen::VectorXd times = trajopt.GetSampleTimes(result);
  EXPECT_EQ(times.size(), 2);
  EXPECT_NEAR(times[0], 0.0, kSolverTolerance);
  EXPECT_NEAR(times[1], 0.1, kSolverTolerance);

  // Now use the constructor which takes a prog.
  MyDirectTrajOpt trajopt2(input, state, time, kNumSampleTimes, kMinTimeStep,
                           kMaxTimeStep, &trajopt.prog());
  EXPECT_EQ(trajopt.prog().num_vars(),
            2 * (input.size() + state.size()) * kNumSampleTimes +
                2 /* time variables */);
}

GTEST_TEST(MultipleShootingTest, VariableTimestepTest) {
  const int kNumInputs{1};
  const int kNumStates{2};
  const int kNumSampleTimes{2};
  const double kMinTimeStep{0.1};
  const double kMaxTimeStep{0.1};
  MyDirectTrajOpt trajopt(kNumInputs, kNumStates, kNumSampleTimes, kMinTimeStep,
                          kMaxTimeStep);

  EXPECT_TRUE(trajopt.time_steps_are_decision_variables());
  EXPECT_THROW(trajopt.fixed_time_step(), std::runtime_error);

  EXPECT_EQ(trajopt.prog().num_vars(), 7);
  const solvers::MathematicalProgramResult result = Solve(trajopt.prog());
  ASSERT_TRUE(result.is_success());
  const Eigen::VectorXd times = trajopt.GetSampleTimes(result);
  EXPECT_EQ(times.size(), 2);
  EXPECT_NEAR(times[0], 0.0, kSolverTolerance);
  EXPECT_NEAR(times[1], 0.1, kSolverTolerance);

  // Now use the constructor which takes a prog.
  MyDirectTrajOpt trajopt2(kNumInputs, kNumStates, kNumSampleTimes,
                           kMinTimeStep, kMaxTimeStep, &trajopt.prog());
  EXPECT_EQ(trajopt.prog().num_vars(), 14);
}

GTEST_TEST(MultipleShootingTest, PlaceholderVariableTest) {
  const int kNumInputs{1};
  const int kNumStates{2};
  const int kNumSampleTimes{2};
  const double kMinTimeStep{0.01};
  const double kMaxTimeStep{1};
  MyDirectTrajOpt trajopt(kNumInputs, kNumStates, kNumSampleTimes, kMinTimeStep,
                          kMaxTimeStep);
  auto& prog = trajopt.prog();

  // Adding a placeholder variable directly to the trajoptram should fail.
  const solvers::VectorDecisionVariable<1>& t = trajopt.time();
  const solvers::VectorXDecisionVariable& u = trajopt.input();
  const solvers::VectorXDecisionVariable& x = trajopt.state();

  EXPECT_THROW(prog.AddCost(t(0)), std::exception);
  EXPECT_THROW(prog.AddLinearCost(Vector1d(1.0), 0.0, u), std::exception);
  EXPECT_THROW(prog.AddQuadraticErrorCost(Eigen::Matrix2d::Identity(),
                                          Eigen::Vector2d::Zero(), x),
               std::exception);

  EXPECT_THROW(prog.AddLinearConstraint(t(0) <= 1.0), std::exception);
  EXPECT_THROW(prog.AddLinearConstraint(u <= Vector1d(1.0)), std::exception);

  EXPECT_THROW(prog.AddLinearConstraint(Eigen::Matrix2d::Identity(),
                                        Eigen::Vector2d::Zero(),
                                        Eigen::Vector2d::Zero(), x),
               std::exception);

  solvers::MathematicalProgramResult result;
  // Arbitrarily set the decision variable values to 0.
  result.set_decision_variable_index(prog.decision_variable_index());
  result.set_x_val(Eigen::VectorXd::Zero(prog.num_vars()));
  EXPECT_THROW(unused(result.GetSolution(t(0))), std::exception);
  EXPECT_THROW(unused(result.GetSolution(u)), std::exception);
}

GTEST_TEST(MultipleShootingTest, PlaceholderVariableNames) {
  const int kNumInputs{1};
  const int kNumStates{2};
  const int kNumSampleTimes{2};
  const double kFixedTimeStep{0.1};
  MyDirectTrajOpt trajopt(kNumInputs, kNumStates, kNumSampleTimes,
                          kFixedTimeStep);

  EXPECT_EQ(trajopt.time().coeff(0).get_name(), "t(0)");
  EXPECT_EQ(trajopt.state().coeff(0).get_name(), "x(0)");
  EXPECT_EQ(trajopt.state().coeff(1).get_name(), "x(1)");
  EXPECT_EQ(trajopt.input().coeff(0).get_name(), "u(0)");
}

GTEST_TEST(MultipleShootingTest, TimeIntervalBoundsTest) {
  const int kNumInputs{1};
  const int kNumStates{1};
  const int kNumSampleTimes{3};
  const double kMinTimeStep{0.01};
  const double kMaxTimeStep{1};
  MyDirectTrajOpt trajopt(kNumInputs, kNumStates, kNumSampleTimes, kMinTimeStep,
                          kMaxTimeStep);

  solvers::Binding<solvers::BoundingBoxConstraint> interval_bounds =
      trajopt.AddTimeIntervalBounds(.5, .5);
  EXPECT_EQ(interval_bounds.evaluator()->lower_bound(),
            Eigen::Vector2d(0.5, 0.5));
  EXPECT_EQ(interval_bounds.evaluator()->upper_bound(),
            Eigen::Vector2d(0.5, 0.5));
  const solvers::MathematicalProgramResult result = Solve(trajopt.prog());
  ASSERT_TRUE(result.is_success());
  EXPECT_TRUE(CompareMatrices(result.GetSolution(trajopt.h_vars()),
                              Eigen::Vector2d(0.5, 0.5), 1e-6));
}

GTEST_TEST(MultipleShootingTest, EqualTimeIntervalsTest) {
  const int kNumInputs{1};
  const int kNumStates{2};
  const int kNumSampleTimes{3};
  const double kMinTimeStep{0.01};
  const double kMaxTimeStep{0.3};
  MyDirectTrajOpt trajopt(kNumInputs, kNumStates, kNumSampleTimes, kMinTimeStep,
                          kMaxTimeStep);
  auto& prog = trajopt.prog();

  std::vector<solvers::Binding<solvers::LinearConstraint>> equal_time_con =
      trajopt.AddEqualTimeIntervalsConstraints();
  EXPECT_EQ(equal_time_con.size(), kNumSampleTimes - 2);

  prog.SetInitialGuess(trajopt.time_step(0), Vector1d(.1));
  prog.SetInitialGuess(trajopt.time_step(1), Vector1d(.2));

  const solvers::MathematicalProgramResult result =
      Solve(prog, prog.initial_guess());
  ASSERT_TRUE(result.is_success());
  EXPECT_NEAR(result.GetSolution(trajopt.time_step(0).coeff(0)),
              result.GetSolution(trajopt.time_step(1).coeff(0)),
              kSolverTolerance);
}

GTEST_TEST(MultipleShootingTest, DurationConstraintTest) {
  const int kNumInputs{0};
  const int kNumStates{1};
  const int kNumSampleTimes{3};
  const double kMinTimeStep{0.01};
  const double kMaxTimeStep{0.3};
  MyDirectTrajOpt trajopt(kNumInputs, kNumStates, kNumSampleTimes, kMinTimeStep,
                          kMaxTimeStep);
  auto& prog = trajopt.prog();

  solvers::Binding<solvers::LinearConstraint> duration_bound =
      trajopt.AddDurationBounds(.5, .5);
  EXPECT_EQ(duration_bound.evaluator()->lower_bound(), Vector1d(0.5));
  EXPECT_EQ(duration_bound.evaluator()->upper_bound(), Vector1d(0.5));

  prog.SetInitialGuess(trajopt.time_step(0), Vector1d(.1));
  prog.SetInitialGuess(trajopt.time_step(1), Vector1d(.2));

  // Note: IPOPT fails with unbounded variables.
  trajopt.AddConstraintToAllKnotPoints(trajopt.state() <= Vector1d(1));
  trajopt.AddConstraintToAllKnotPoints(trajopt.state() >= Vector1d(0));

  const solvers::MathematicalProgramResult result =
      Solve(prog, prog.initial_guess());
  ASSERT_TRUE(result.is_success());
  EXPECT_NEAR(result.GetSolution(trajopt.h_vars()).sum(), .5, 1e-6);
}

GTEST_TEST(MultipleShootingTest, ConstraintAllKnotsTest) {
  const int kNumInputs{1};
  const int kNumStates{2};
  const int kNumSampleTimes{3};
  const double kMinTimeStep{0.01};
  const double kMaxTimeStep{1};
  MyDirectTrajOpt trajopt(kNumInputs, kNumStates, kNumSampleTimes, kMinTimeStep,
                          kMaxTimeStep);

  const Eigen::Vector2d state_value(4.0, 5.0);
  std::vector<solvers::Binding<solvers::Constraint>> state_con =
      trajopt.AddConstraintToAllKnotPoints(trajopt.state() == state_value);
  EXPECT_EQ(state_con.size(), kNumSampleTimes);

  solvers::MathematicalProgramResult result = Solve(trajopt.prog());
  ASSERT_TRUE(result.is_success());
  for (int i = 0; i < kNumSampleTimes; i++) {
    // osqp can fail in polishing step, such that the accuracy cannot reach
    // 1E-6.
    const double tol =
        result.get_solver_id() == solvers::OsqpSolver::id() ? 4E-6 : 1E-6;
    EXPECT_TRUE(CompareMatrices(result.GetSolution(trajopt.state(i)),
                                state_value, tol));
  }

  const solvers::VectorDecisionVariable<1>& t = trajopt.time();
  const solvers::VectorXDecisionVariable& u = trajopt.input();
  trajopt.AddConstraintToAllKnotPoints(u == t);
  result = Solve(trajopt.prog());
  ASSERT_TRUE(result.is_success());
  // u(0) = 0.
  EXPECT_NEAR(result.GetSolution(trajopt.input(0).coeff(0)), 0.0, 1e-6);
  // u(1) = h(0).
  EXPECT_NEAR(result.GetSolution(trajopt.input(1).coeff(0)),
              result.GetSolution(trajopt.time_step(0).coeff(0)), 1e-6);
  // u(2) = h(0)+h(1).
  EXPECT_NEAR(result.GetSolution(trajopt.input(2).coeff(0)),
              result.GetSolution(trajopt.h_vars()).sum(), 1e-6);

  // Add the same constraint again using the shared_ptr variant.
  std::vector<solvers::Binding<solvers::BoundingBoxConstraint>> c =
      trajopt.AddConstraintToAllKnotPoints(
          std::make_shared<solvers::BoundingBoxConstraint>(state_value,
                                                           state_value),
          trajopt.state());
  EXPECT_EQ(c.size(), kNumSampleTimes);

  // Add the same constraint again using the vector of formulas variant.
  std::vector<solvers::Binding<solvers::Constraint>> c2 =
      trajopt.AddConstraintToAllKnotPoints(
          Vector2<symbolic::Formula>{trajopt.state()[0] == state_value[0],
                                     trajopt.state()[1] == state_value[1]});
  EXPECT_EQ(c2.size(), 2 * kNumSampleTimes);
}

GTEST_TEST(MultipleShootingTest, FinalCostTest) {
  const int kNumInputs{1};
  const int kNumStates{2};
  const int kNumSampleTimes{2};
  const double kFixedTimeStep{0.1};
  MyDirectTrajOpt trajopt(kNumInputs, kNumStates, kNumSampleTimes,
                          kFixedTimeStep);

  // Set desired value for the final state.
  const Eigen::Vector2d desired_state(2.0, 3.0);
  const auto error = trajopt.state() - desired_state;

  solvers::Binding<solvers::Cost> final_cost =
      trajopt.AddFinalCost(error.dot(error));
  const solvers::MathematicalProgramResult result = Solve(trajopt.prog());
  ASSERT_TRUE(result.is_success());
  EXPECT_NEAR(result.get_optimal_cost(), 0.0, kSolverTolerance);
  EXPECT_TRUE(CompareMatrices(result.EvalBinding(final_cost), Vector1d(0.0),
                              kSolverTolerance));
  EXPECT_TRUE(CompareMatrices(result.GetSolution(trajopt.state(1)),
                              desired_state, kSolverTolerance));
}

GTEST_TEST(MultipleShootingTest, TrajectoryCallbackTest) {
  const int kNumInputs{1};
  const int kNumStates{2};
  const int kNumSampleTimes{3};
  const double kFixedTimeStep{0.1};
  const std::vector<std::string> extra_variables{"v", "w"};

  bool input_callback_was_called = false;
  auto my_input_callback = [&input_callback_was_called](
                               const Eigen::Ref<const Eigen::VectorXd>& t,
                               const Eigen::Ref<const Eigen::MatrixXd>& u) {
    EXPECT_TRUE(CompareMatrices(t, Eigen::Vector3d(0., .1, .2)));
    EXPECT_TRUE(CompareMatrices(u, Eigen::RowVector3d(1., 2., 3.)));
    input_callback_was_called = true;
  };
  bool state_callback_was_called = false;
  auto my_state_callback = [&state_callback_was_called](
                               const Eigen::Ref<const Eigen::VectorXd>& t,
                               const Eigen::Ref<const Eigen::MatrixXd>& x) {
    EXPECT_TRUE(CompareMatrices(t, Eigen::Vector3d(0., .1, .2)));
    Eigen::MatrixXd x_expected(2, 3);
    // clang-format off
    x_expected << 4., 6., 8.,
                  5., 7., 9.;
    // clang-format on
    EXPECT_TRUE(CompareMatrices(x, x_expected));
    state_callback_was_called = true;
  };
  bool complete_callback_was_called = false;
  auto my_complete_callback =
      [&complete_callback_was_called](
          const Eigen::Ref<const Eigen::VectorXd>& t,
          const Eigen::Ref<const Eigen::MatrixXd>& x,
          const Eigen::Ref<const Eigen::MatrixXd>& u,
          const std::vector<Eigen::Ref<const Eigen::MatrixXd>>& v) {
        EXPECT_TRUE(CompareMatrices(t, Eigen::Vector3d(0., .1, .2)));
        Eigen::MatrixXd x_expected(2, 3);
        // clang-format off
        x_expected << 4., 6., 8.,
                      5., 7., 9.;
        // clang-format on
        EXPECT_TRUE(CompareMatrices(x, x_expected));
        EXPECT_TRUE(CompareMatrices(u, Eigen::RowVector3d(1., 2., 3.)));
        EXPECT_TRUE(CompareMatrices(v[0], Eigen::RowVector3d(10., 11., 12.)));
        EXPECT_TRUE(CompareMatrices(v[1], Eigen::RowVector3d(13., 14., 15.)));
        complete_callback_was_called = true;
      };

  // Test *without* time steps as decision variables.
  MyDirectTrajOpt trajopt(kNumInputs, kNumStates, kNumSampleTimes,
                          kFixedTimeStep);
  auto& prog = trajopt.prog();
  prog.SetInitialGuess(trajopt.u_vars(), Eigen::Vector3d::LinSpaced(3, 1., 3.));
  prog.SetInitialGuess(trajopt.x_vars(), Eigen::VectorXd::LinSpaced(6, 4., 9.));

  solvers::Binding<solvers::VisualizationCallback> b =
      trajopt.AddInputTrajectoryCallback(my_input_callback);
  EXPECT_EQ(prog.visualization_callbacks().size(), 1);
  input_callback_was_called = false;
  prog.EvalBindingAtInitialGuess(b);
  EXPECT_TRUE(input_callback_was_called);

  b = trajopt.AddStateTrajectoryCallback(my_state_callback);
  EXPECT_EQ(prog.visualization_callbacks().size(), 2);
  state_callback_was_called = false;
  prog.EvalBindingAtInitialGuess(b);
  EXPECT_TRUE(state_callback_was_called);

  trajopt.NewSequentialVariable(1, extra_variables[0]);
  trajopt.NewSequentialVariable(1, extra_variables[1]);
  prog.SetInitialGuess(trajopt.GetSequentialVariable(extra_variables[0]),
                       Eigen::Vector3d::LinSpaced(3, 10., 12.));
  prog.SetInitialGuess(trajopt.GetSequentialVariable(extra_variables[1]),
                       Eigen::Vector3d::LinSpaced(3, 13., 15.));
  b = trajopt.AddCompleteTrajectoryCallback(my_complete_callback,
                                            extra_variables);
  EXPECT_EQ(prog.visualization_callbacks().size(), 3);
  complete_callback_was_called = false;
  prog.EvalBindingAtInitialGuess(b);
  EXPECT_TRUE(complete_callback_was_called);

  // Test with time steps as decision variables.
  MyDirectTrajOpt trajopt2(kNumInputs, kNumStates, kNumSampleTimes,
                           kFixedTimeStep, kFixedTimeStep);
  auto& prog2 = trajopt2.prog();
  prog2.SetInitialGuess(trajopt2.h_vars(),
                        Eigen::Vector2d::Constant(kFixedTimeStep));
  prog2.SetInitialGuess(trajopt2.u_vars(),
                        Eigen::Vector3d::LinSpaced(3, 1., 3.));
  prog2.SetInitialGuess(trajopt2.x_vars(),
                        Eigen::VectorXd::LinSpaced(6, 4., 9.));

  b = trajopt2.AddInputTrajectoryCallback(my_input_callback);
  EXPECT_EQ(prog2.visualization_callbacks().size(), 1);
  input_callback_was_called = false;
  prog2.EvalBindingAtInitialGuess(b);
  EXPECT_TRUE(input_callback_was_called);

  b = trajopt2.AddStateTrajectoryCallback(my_state_callback);
  EXPECT_EQ(prog2.visualization_callbacks().size(), 2);
  state_callback_was_called = false;
  prog2.EvalBindingAtInitialGuess(b);
  EXPECT_TRUE(state_callback_was_called);

  trajopt2.NewSequentialVariable(1, extra_variables[0]);
  trajopt2.NewSequentialVariable(1, extra_variables[1]);
  prog2.SetInitialGuess(trajopt2.GetSequentialVariable(extra_variables[0]),
                        Eigen::Vector3d::LinSpaced(3, 10., 12.));
  prog2.SetInitialGuess(trajopt2.GetSequentialVariable(extra_variables[1]),
                        Eigen::Vector3d::LinSpaced(3, 13., 15.));
  b = trajopt2.AddCompleteTrajectoryCallback(my_complete_callback,
                                             extra_variables);
  EXPECT_EQ(prog2.visualization_callbacks().size(), 3);
  complete_callback_was_called = false;
  prog2.EvalBindingAtInitialGuess(b);
  EXPECT_TRUE(complete_callback_was_called);
}

GTEST_TEST(MultipleShootingTest, InitialGuessTest) {
  const int kNumInputs{1};
  const int kNumStates{1};
  const int kNumSampleTimes{3};
  const double kMinTimeStep{0.01};
  const double kMaxTimeStep{0.5};
  MyDirectTrajOpt trajopt(kNumInputs, kNumStates, kNumSampleTimes, kMinTimeStep,
                          kMaxTimeStep);
  auto& prog = trajopt.prog();

  const auto traj1 = PiecewisePolynomial<double>::FirstOrderHold(
      {1, 2}, {Vector1d(3), Vector1d(4)});
  const auto traj2 = PiecewisePolynomial<double>::FirstOrderHold(
      {1, 4}, {Vector1d(5), Vector1d(6)});

  // Throws if both are empty.
  EXPECT_THROW(trajopt.SetInitialTrajectory(PiecewisePolynomial<double>(),
                                            PiecewisePolynomial<double>()),
               std::runtime_error);

  // If one is empty, uses the duration.
  trajopt.SetInitialTrajectory(PiecewisePolynomial<double>(), traj1);
  // Pretends that the solver has solved the optimization problem, and sets
  // the solution to trajopt.initial_guess().
  solvers::MathematicalProgramResult result;
  result.set_solver_id(solvers::SolverId("dummy"));
  result.set_decision_variable_index(prog.decision_variable_index());
  result.set_x_val(prog.initial_guess());
  EXPECT_EQ(trajopt.GetSampleTimes(result), Eigen::Vector3d(0.0, 0.5, 1.0));

  trajopt.SetInitialTrajectory(traj2, PiecewisePolynomial<double>());
  // Pretends that the solver has solved the optimization problem, and sets
  // the solution to trajopt.initial_guess().
  result.set_x_val(prog.initial_guess());
  EXPECT_EQ(trajopt.GetSampleTimes(result), Eigen::Vector3d(0.0, 1.5, 3.0));

  trajopt.SetInitialTrajectory(traj1, traj1);
  // Pretends that the solver has solved the optimization problem, and sets
  // the solution to trajopt.initial_guess().
  result.set_x_val(prog.initial_guess());
  EXPECT_EQ(trajopt.GetSampleTimes(result), Eigen::Vector3d(0.0, 0.5, 1.0));

  // Throws if trajectories don't match.
  EXPECT_THROW(trajopt.SetInitialTrajectory(traj1, traj2), std::runtime_error);
}

GTEST_TEST(MultipleShootingTest, ResultSamplesTest) {
  const int kNumInputs{1};
  const int kNumStates{2};
  const int kNumSampleTimes{2};
  const double kFixedTimeStep{0.1};
  MyDirectTrajOpt trajopt(kNumInputs, kNumStates, kNumSampleTimes,
                          kFixedTimeStep);
  auto& prog = trajopt.prog();

  const Eigen::RowVector2d input_trajectory(1, 2);
  Eigen::Matrix2d state_trajectory;
  // clang-format off
  state_trajectory << 3, 4,
                      5, 6;
  // clang-format on

  for (int i = 0; i < kNumSampleTimes; i++) {
    prog.SetInitialGuess(trajopt.input(i), input_trajectory.col(i));
    prog.SetInitialGuess(trajopt.state(i), state_trajectory.col(i));
  }
  // Pretends that the solver has solved the optimization problem, and sets
  // the solution to trajopt.initial_guess().
  solvers::MathematicalProgramResult result;
  result.set_solver_id(solvers::SolverId("dummy"));
  result.set_decision_variable_index(prog.decision_variable_index());
  result.set_x_val(prog.initial_guess());

  EXPECT_TRUE(CompareMatrices(trajopt.GetSampleTimes(result),
                              Eigen::Vector2d(0.0, kFixedTimeStep), 0.0));
  EXPECT_TRUE(
      CompareMatrices(trajopt.GetInputSamples(result), input_trajectory, 0.0));
  EXPECT_TRUE(
      CompareMatrices(trajopt.GetStateSamples(result), state_trajectory, 0.0));
}

GTEST_TEST(MultipleShootingTest, NewSequentialVariableTest) {
  const int kNumInputs{1};
  const int kNumStates{2};
  const int kNumSampleTimes{3};
  const double kMinTimeStep{0.01};
  const double kMaxTimeStep{1};
  MyDirectTrajOpt trajopt(kNumInputs, kNumStates, kNumSampleTimes, kMinTimeStep,
                          kMaxTimeStep);
  const Eigen::Vector2d state_value(4.0, 5.0);
  trajopt.AddConstraintToAllKnotPoints(trajopt.state() == state_value);

  solvers::VectorXDecisionVariable new_sequential_variable =
      trajopt.NewSequentialVariable(kNumStates, "w");
  trajopt.AddConstraintToAllKnotPoints(-2.0 * new_sequential_variable ==
                                       trajopt.state());

  solvers::MathematicalProgramResult result = Solve(trajopt.prog());
  ASSERT_TRUE(result.is_success());
  // osqp can fail in polishing step, such that the accuracy cannot reach
  // 1E-6.
  const double tol =
      result.get_solver_id() == solvers::OsqpSolver::id() ? 4E-6 : 1E-6;
  for (int i = 0; i < kNumSampleTimes; i++) {
    // Verify that GetSequentialVariableAtIndex() works as expected.
    EXPECT_TRUE(CompareMatrices(
        -2.0 * result.GetSolution(trajopt.GetSequentialVariableAtIndex("w", i)),
        state_value, tol));
  }

  EXPECT_TRUE(
      CompareMatrices(-2.0 * trajopt.GetSequentialVariableSamples(result, "w"),
                      trajopt.GetStateSamples(result), tol));

  const solvers::VectorDecisionVariable<1>& t = trajopt.time();
  const solvers::VectorXDecisionVariable& u = trajopt.input();
  trajopt.AddConstraintToAllKnotPoints(u == t);
  result = Solve(trajopt.prog());
  ASSERT_TRUE(result.is_success());
  // u(0) = 0.
  EXPECT_NEAR(result.GetSolution(trajopt.input(0).coeff(0)), 0.0, 1e-6);
  // u(1) = h(0).
  EXPECT_NEAR(result.GetSolution(trajopt.input(1).coeff(0)),
              result.GetSolution(trajopt.time_step(0).coeff(0)), 1e-6);
  // u(2) = h(0)+h(1).
  EXPECT_NEAR(result.GetSolution(trajopt.input(2).coeff(0)),
              result.GetSolution(trajopt.h_vars()).sum(), 1e-6);
}

}  // namespace
}  // namespace trajectory_optimization
}  // namespace planning
}  // namespace drake
