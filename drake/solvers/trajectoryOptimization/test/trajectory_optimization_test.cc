
#include <vector>

#include "gtest/gtest.h"
#include "drake/solvers/trajectoryOptimization/direct_trajectory_optimization.h"
#include "drake/systems/trajectories/PiecewisePolynomial.h"
#include "drake/systems/vector.h"

#include "drake/util/eigen_matrix_compare.h"

using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

using drake::util::MatrixCompareType;

namespace drake {
namespace solvers {
namespace {

typedef PiecewisePolynomial<double> PiecewisePolynomialType;

GTEST_TEST(TrajectoryOptimizationTest, DirectTrajectoryOptimizationTest) {
  const int kNumInputs(1);
  const int kNumStates(2);
  const int kNumTimeSamples(21);  // aka N.
  DirectTrajectoryOptimization direct_traj(kNumInputs, kNumStates,
                                           kNumTimeSamples, 0, 25);

  const double t_init_in(7);
  const Polynomiald y = Polynomiald("y");
  const Polynomiald y1 = (7 * y);
  const Polynomiald y2 = (2 * y) + 1;

  const std::vector<Polynomiald> u_vec{y1};
  std::vector<PiecewisePolynomialType::PolynomialMatrix> y_vec(1);
  y_vec[0].resize(2, 1);
  y_vec[0](0) = y1;
  y_vec[0](1) = y2;
  const std::vector<double> times{0.0, t_init_in};

  const PiecewisePolynomialType inputs_u(u_vec, times);
  const PiecewisePolynomialType states_x(y_vec, times);

  const Vector1d constrained_input(30);
  auto input_constraint = std::make_shared<LinearEqualityConstraint>(
      Vector1d(1), constrained_input);
  direct_traj.AddInputConstraint(input_constraint,
                                 {1, kNumTimeSamples - 2});

  const Eigen::Vector2d constrained_state(11, 22);
  auto state_constraint = std::make_shared<LinearEqualityConstraint>(
      Eigen::Matrix2d::Identity(), constrained_state);
  direct_traj.AddStateConstraint(state_constraint,
                                 {0, kNumTimeSamples - 1});

  SolutionResult result = SolutionResult::kUnknownError;
  result =
      direct_traj.SolveTraj(t_init_in, PiecewisePolynomialType(), states_x);
  EXPECT_EQ(result, SolutionResult::kSolutionFound) << "Result is an Error";

  Eigen::MatrixXd inputs;
  Eigen::MatrixXd states;
  std::vector<double> times_out;

  direct_traj.GetResultSamples(&inputs, &states, &times_out);
  PiecewisePolynomial<double> input_traj =
      direct_traj.ReconstructInputTrajectory();

  EXPECT_TRUE(
      CompareMatrices(constrained_input, inputs.col(1),
                      1e-10, MatrixCompareType::absolute));
  EXPECT_TRUE(
      CompareMatrices(constrained_input, input_traj.value(times_out[1]),
                      1e-10, MatrixCompareType::absolute));
  EXPECT_TRUE(
      CompareMatrices(constrained_input, inputs.col(kNumTimeSamples - 2),
                      1e-10, MatrixCompareType::absolute));
  EXPECT_TRUE(
      CompareMatrices(constrained_input,
                      input_traj.value(times_out[kNumTimeSamples - 2]),
                      1e-10, MatrixCompareType::absolute));

  PiecewisePolynomial<double> state_traj =
      direct_traj.ReconstructStateTrajectory();
  EXPECT_TRUE(
      CompareMatrices(constrained_state, states.col(0),
                      1e-10, MatrixCompareType::absolute));
  EXPECT_TRUE(
      CompareMatrices(constrained_state, state_traj.value(times_out[0]),
                      1e-10, MatrixCompareType::absolute));
  EXPECT_TRUE(
      CompareMatrices(constrained_state, states.col(kNumTimeSamples - 1),
                      1e-10, MatrixCompareType::absolute));
  EXPECT_TRUE(
      CompareMatrices(constrained_state,
                      state_traj.value(times_out[kNumTimeSamples - 1]),
                      1e-10, MatrixCompareType::absolute));

  result = direct_traj.SolveTraj(t_init_in, inputs_u, states_x);
  EXPECT_EQ(result, SolutionResult::kSolutionFound) << "Result is an Error";
}

}  // anonymous namespace
}  // namespace solvers
}  // namespace drake
