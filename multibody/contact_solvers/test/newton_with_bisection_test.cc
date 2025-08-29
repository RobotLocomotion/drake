#include "drake/multibody/contact_solvers/newton_with_bisection.h"

#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
namespace {

using Eigen::Vector3d;

constexpr double kEpsilon = std::numeric_limits<double>::epsilon();

typedef std::function<std::pair<double, double>(double)> Function;

// Struct used to store test data.
struct RootFindingTestData {
  // Google Test uses this operator to report the test data in the log file
  // when a test fails.
  friend std::ostream& operator<<(std::ostream& os,
                                  const RootFindingTestData& data) {
    return os << "{\n"
              << "  Function: " << data.description << std::endl
              << "  [a , b ] = [" << data.a << ", " << data.b << "]"
              << std::endl
              << "  guess = " << data.guess << std::endl
              << "  [fa, fb] = [" << data.function(data.a).first << ", "
              << data.function(data.b).second << "]" << std::endl
              << "  Root: " << data.root << std::endl
              << "  Root tolerance: " << data.x_tolerance << std::endl
              << "  Function tolerance: " << data.f_tolerance << std::endl
              << "}" << std::flush;
  }

  // Text description of the function, typically in the format y(x) = a * x + b.
  std::string description;
  // The function to find the root within interval [a, b].
  Function function;
  double a{NAN}, b{NAN};  // Interval used for root finding.
  double guess{NAN};      // The initial guess. It must be in [a, b].
  double root{NAN};       // The true root in [a, b].
  double x_tolerance{5.0 * kEpsilon};  // Absolute tolerance for x.
  double f_tolerance{5.0 * kEpsilon};  // Absolute tolerance for the function f.
  int max_iterations{100};             // Solver maximum number of iterations.
  // Expected number of iterations, if known.
  std::optional<int> num_iterations;
};

std::vector<RootFindingTestData> GenerateTestCases() {
  std::vector<RootFindingTestData> cases;
  cases.push_back({.description = "y = 1.5 * x + 3.0",
                   .function =
                       [](double x) {
                         double f = 1.5 * x + 3.0;
                         double dfx = 1.5;
                         return std::make_pair(f, dfx);
                       },
                   .a = -4.0,
                   .b = 3.0,
                   .guess = -0.5,
                   .root = -2.0,
                   .x_tolerance = kEpsilon,
                   .f_tolerance = kEpsilon,
                   .max_iterations = 100,
                   // Since the function is linear, it reaches the solution in
                   // the first iteration (one evaluation). An additional
                   // evaluation is needed to verify convergence.
                   .num_iterations = 2});

  // This function has two roots. We push them as two separate cases with two
  // different search intervals.
  cases.push_back({.description = "y = (x - 1.5) * (x + 2.0) = x² + 0.5 x − 3",
                   .function =
                       [](double x) {
                         double f = (x - 1.5) * (x + 2.0);
                         double dfx = 2.0 * x + 0.5;
                         return std::make_pair(f, dfx);
                       },
                   .a = -1.0,
                   .b = 2.0,
                   .guess = 1.0,
                   .root = 1.5});
  cases.push_back({.description = "y = (x - 1.5) * (x + 2.0) = x² + 0.5 x − 3",
                   .function =
                       [](double x) {
                         double f = (x - 1.5) * (x + 2.0);
                         double dfx = 2.0 * x + 0.5;
                         return std::make_pair(f, dfx);
                       },
                   .a = -4.0,
                   .b = 1.0,
                   .guess = 1.0,
                   .root = -2.0});

  cases.push_back({.description = "y = arctan(x)",
                   .function =
                       [](double x) {
                         double f = std::atan(x);
                         double dfx = 1.0 / (1.0 + x * x);
                         return std::make_pair(f, dfx);
                       },
                   .a = -1.0,
                   .b = 10.0,
                   .guess = 9.0,
                   .root = 0.0});

  // For y = x³ − 2x + 2 with initial guess x0 = 0, Newton-Raphson will enter
  // into an infinity cycle without convergence.
  cases.push_back({.description = "y = x³ − 2x + 2",
                   .function =
                       [](double x) {
                         const double x2 = x * x;
                         const double f = x * (x2 - 2) + 2;
                         const double dfx = 3 * x2 - 2;
                         return std::make_pair(f, dfx);
                       },
                   .a = -3.0,
                   .b = 3.0,
                   .guess = 0.0,
                   .root = -1.769292354238631415240409464335});

  // This is a difficult case since y' = ∞ at the root x = 0.
  cases.push_back({.description = "y = sign(x) * abs(x)^(1/2)",
                   .function =
                       [](double x) {
                         const double s = x >= 0 ? 1.0 : -1.0;
                         const double sqrt_x = std::sqrt(std::abs(x));
                         const double f = s * sqrt_x;
                         const double dfx = 0.5 / sqrt_x;
                         return std::make_pair(f, dfx);
                       },
                   .a = -1.0,
                   .b = 4.0,
                   .guess = 2.0,
                   .root = 0.0});

  // For y = x - tan(x) Newton-Raphson will diverge if the guess is outside
  // [4.3, 4.7].
  cases.push_back({.description = "y = x - tan(x)",
                   .function =
                       [](double x) {
                         const double tan_x = std::tan(x);
                         const double f = x - tan_x;
                         const double dfx = -tan_x * tan_x;
                         return std::make_pair(f, dfx);
                       },
                   .a = 2.0,
                   .b = 4.7,
                   .guess = 3.0,
                   .root = 4.4934094579090641753078809272803});

  // Newton-Raphson struggles with this case since the derivative also goes to
  // zero at the (triple) root x = 1.5.
  cases.push_back({.description = "y = (x-1.5)³",
                   .function =
                       [](double x) {
                         const double arg = x - 1.5;
                         const double f = arg * arg * arg;
                         const double dfx = 3.0 * arg * arg;
                         return std::make_pair(f, dfx);
                       },
                   .a = 0.0,
                   .b = 2.0,
                   .guess = 1.0,
                   .root = 1.5});

  // Function with discontinuous derivative at the x = 1.0. The guess is
  // provided at x = 1.5, so that the iteration must go through the
  // discontinuity.
  cases.push_back({.description = "y = x < 1.0 ? x : 2 * x",
                   .function =
                       [](double x) {
                         if (x < 1.0) {
                           return std::make_pair(x, 1.0);
                         }
                         return std::make_pair(2.0 * x, 2.0);
                       },
                   .a = -1.0,
                   .b = 2.5,
                   .guess = 1.5,
                   .root = 0.0});

  // Same as the function above with discontinuous derivative at the x = 1.0.
  // The guess this time is at x = 0.5, and therefore we expect to converge in
  // exactly a single iteration (the function is linear).
  cases.push_back({.description = "y = x < 1.0 ? x : 2 * x",
                   .function =
                       [](double x) {
                         if (x < 1.0) {
                           return std::make_pair(x, 1.0);
                         }
                         return std::make_pair(2.0 * x, 2.0);
                       },
                   .a = -1.0,
                   .b = 2.5,
                   .guess = 0.5,
                   .root = 0.0,
                   .x_tolerance = kEpsilon,
                   .f_tolerance = kEpsilon,
                   .max_iterations = 100,
                   // Since the function is linear, it reaches the solution in
                   // the first iteration (one evaluation). An additional
                   // evaluation is needed to verify convergence.
                   .num_iterations = 2});

  // This case is setup so that the derivative of the function at the initial
  // guess is zero, and therefore the Newton direction is not well defined.
  cases.push_back({.description = "y = x * (x - 1)",
                   .function =
                       [](double x) {
                         const double f = x * (x - 1.0);
                         const double dfx = 2.0 * x - 1.0;
                         return std::make_pair(f, dfx);
                       },
                   .a = -1.0,
                   .b = 0.6,
                   .guess = 0.5,
                   .root = 0.0});

  return cases;
}

// Test parametrized on different root finding cases.
// To see debug information printed out by DoNewtonWithBisectionFallback, run
// with:
//   bazel run -c dbg multibody/contact_solvers:newton_with_bisection_test --
//   --spdlog_level debug
struct RootFindingTest : public testing::TestWithParam<RootFindingTestData> {};

TEST_P(RootFindingTest, VerifyExpectedResults) {
  const RootFindingTestData& data = GetParam();

  // We printout the human-readable description so that when running the tests
  // we see more than Test/0, Test/1, etc.
  std::cout << data << std::endl;

  // Create bracket.
  const Bracket bracket(data.a, data.function(data.a).first, data.b,
                        data.function(data.b).first);

  // Find root in the interval [data.a, data.b]
  const auto [x, num_iterations] = DoNewtonWithBisectionFallback(
      data.function, bracket, data.guess, data.x_tolerance, data.f_tolerance,
      data.max_iterations);

  // Verify DoNewtonWithBisectionFallback converged.
  const bool root_convergence = std::abs(x - data.root) < data.x_tolerance;
  const bool function_convergence =
      std::abs(data.function(x).first) < data.f_tolerance;
  EXPECT_TRUE(root_convergence || function_convergence);

  // If we have an expected number of iterations, verify it.
  if (data.num_iterations) {
    EXPECT_EQ(num_iterations, *data.num_iterations);
  }
}

// To debug a specific test, you can use Bazel flag --test_filter and
// --test_output.  For example, you can use the command:
//   bazel run //multibody/contact_solvers:newton_with_bisection_test
//   --test_filter=AllCases/RootFindingTest.VerifyExpectedResults/0
//   --test_output=all
INSTANTIATE_TEST_SUITE_P(AllCases, RootFindingTest,
                         testing::ValuesIn(GenerateTestCases()));

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
