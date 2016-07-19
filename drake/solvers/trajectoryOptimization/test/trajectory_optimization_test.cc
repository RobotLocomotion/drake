#include <random>
#include <vector>

#include "gtest/gtest.h"
#include "drake/solvers/trajectoryOptimization/direct_trajectory_optimization.h"
#include "drake/systems/trajectories/PiecewisePolynomial.h"

using std::vector;

namespace drake {
namespace solvers {
namespace {

typedef PiecewisePolynomial<double> PiecewisePolynomialType;

GTEST_TEST(TrajectoryOptimizationTest, DirectTrajectoryOptimizationTest) {
  const int kNumInputs(1);
  const int kNumStates(2);
  const int kNumTimeSamples(21);  // aka N.
  DirectTrajectoryOptimization directTraj(kNumInputs, kNumStates,
                                          kNumTimeSamples, 0, 25);

  const double t_init_in(7);
  const VectorXd s_times{VectorXd::LinSpaced(kNumTimeSamples, 0, t_init_in)};
  const vector<double> segment_times(
      s_times.data(), s_times.data() + s_times.rows() * s_times.cols());

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
  ASSERT_NO_THROW(result = directTraj.SolveTraj(
                      t_init_in, PiecewisePolynomialType(), states_x));
  EXPECT_EQ(result, SolutionResult::kSolutionFound) << "Result is an Error";

  ASSERT_NO_THROW(result = directTraj.SolveTraj(t_init_in, inputs_u, states_x));
  EXPECT_EQ(result, SolutionResult::kSolutionFound) << "Result is an Error";
}

}  // anonymous namespace
}  // namespace solvers
}  // namespace drake
