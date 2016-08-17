#include <vector>

#include "gtest/gtest.h"

#include "drake/common/eigen_matrix_compare.h"
#include "drake/solvers/trajectoryOptimization/direct_trajectory_optimization.h"
#include "drake/systems/trajectories/PiecewisePolynomial.h"
#include "drake/systems/vector.h"

using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

using drake::solvers::detail::VecIn;
using drake::solvers::detail::VecOut;

namespace drake {
namespace solvers {
namespace {

typedef PiecewisePolynomial<double> PiecewisePolynomialType;

class InitialCost {
 public:
  static size_t numInputs() { return 2; }
  static size_t numOutputs() { return 1; }

  template <typename ScalarType>
  void eval(VecIn<ScalarType> const& x, VecOut<ScalarType>& y) const {
    y(0) = x(1) * x(1);
  }
};

class FinalCost {
 public:
  static size_t numInputs() { return 3; }
  static size_t numOutputs() { return 1; }

  template <typename ScalarType>
  void eval(VecIn<ScalarType> const& x, VecOut<ScalarType>& y) const {
    y(0) = x(2) * x(2);
  }
};

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

  const int kInputConstraintLo = 1;
  const int kInputConstraintHi = kNumTimeSamples - 2;
  const Vector1d constrained_input(30);
  auto input_constraint = std::make_shared<LinearEqualityConstraint>(
      Vector1d(1), constrained_input);
  direct_traj.AddInputConstraint(input_constraint,
                                 {kInputConstraintLo, kInputConstraintHi});

  const int kStateConstraintLo = 2;
  const int kStateConstraintHi = kNumTimeSamples - 3;
  const Eigen::Vector2d constrained_state(11, 22);
  auto state_constraint = std::make_shared<LinearEqualityConstraint>(
      Eigen::Matrix2d::Identity(), constrained_state);
  direct_traj.AddStateConstraint(state_constraint,
                                 {kStateConstraintLo, kStateConstraintHi});

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
      CompareMatrices(constrained_input, inputs.col(kInputConstraintLo),
                      1e-10, MatrixCompareType::absolute));
  EXPECT_TRUE(
      CompareMatrices(constrained_input,
                      input_traj.value(times_out[kInputConstraintLo]),
                      1e-10, MatrixCompareType::absolute));
  EXPECT_TRUE(
      CompareMatrices(constrained_input, inputs.col(kInputConstraintHi),
                      1e-10, MatrixCompareType::absolute));
  EXPECT_TRUE(
      CompareMatrices(constrained_input,
                      input_traj.value(times_out[kInputConstraintHi]),
                      1e-10, MatrixCompareType::absolute));

  PiecewisePolynomial<double> state_traj =
      direct_traj.ReconstructStateTrajectory();
  EXPECT_TRUE(
      CompareMatrices(constrained_state, states.col(kStateConstraintLo),
                      1e-10, MatrixCompareType::absolute));
  EXPECT_TRUE(
      CompareMatrices(constrained_state,
                      state_traj.value(times_out[kStateConstraintLo]),
                      1e-10, MatrixCompareType::absolute));
  EXPECT_TRUE(
      CompareMatrices(constrained_state, states.col(kStateConstraintHi),
                      1e-10, MatrixCompareType::absolute));
  EXPECT_TRUE(
      CompareMatrices(constrained_state,
                      state_traj.value(times_out[kStateConstraintHi]),
                      1e-10, MatrixCompareType::absolute));

  // Add bounds on the inputs and make sure they're enforced.
  Vector1d input_min(1);
  direct_traj.AddInputBounds(input_min, constrained_input);
  result =
      direct_traj.SolveTraj(t_init_in, PiecewisePolynomialType(), states_x);
  EXPECT_EQ(result, SolutionResult::kSolutionFound) << "Result is an Error";
  direct_traj.GetResultSamples(&inputs, &states, &times_out);

  EXPECT_GE(inputs(0, 0), input_min(0));

  result = direct_traj.SolveTraj(t_init_in, inputs_u, states_x);
  EXPECT_EQ(result, SolutionResult::kSolutionFound) << "Result is an Error";

  // Add some cost functions and see that something gets minimized.
  // First check that we have values not particularly near zero where
  // we're going to try to minimize next time.
  direct_traj.GetResultSamples(&inputs, &states, &times_out);
  EXPECT_DOUBLE_EQ(states(1, 0), 1);
  EXPECT_DOUBLE_EQ(states(1, kNumTimeSamples - 1), 15);

  direct_traj.AddInitialCost(InitialCost());
  direct_traj.AddFinalCost(FinalCost());
  result = direct_traj.SolveTraj(t_init_in, inputs_u, states_x);
  EXPECT_EQ(result, SolutionResult::kSolutionFound) << "Result is an Error";

  direct_traj.GetResultSamples(&inputs, &states, &times_out);
  EXPECT_NEAR(states(1, 0), 0, 1e-10);
  EXPECT_NEAR(states(1, kNumTimeSamples - 1), 0, 1e-10);
}

}  // anonymous namespace
}  // namespace solvers
}  // namespace drake
