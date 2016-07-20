#include <random>
#include <vector>

#include "gtest/gtest.h"
#include "drake/solvers/trajectoryOptimization/direct_trajectory_optimization.h"
#include "drake/systems/trajectories/PiecewisePolynomial.h"

using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

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

  SolutionResult result = SolutionResult::kUnknownError;
  result =
      direct_traj.SolveTraj(t_init_in, PiecewisePolynomialType(), states_x);
  EXPECT_EQ(result, SolutionResult::kSolutionFound) << "Result is an Error";

  result = direct_traj.SolveTraj(t_init_in, inputs_u, states_x);
  EXPECT_EQ(result, SolutionResult::kSolutionFound) << "Result is an Error";
}

}  // anonymous namespace
}  // namespace solvers
}  // namespace drake
