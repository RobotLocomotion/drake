#include "drake/multibody/contact_solvers/newton_with_bisection.h"

#include <optional>

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
              << "  Tolerance: " << data.abs_tolerance << std::endl
              << "}" << std::flush;
  }

  // Text description of the function, typically in the format y(x) = a * x + b.
  std::string description;
  // The function to find the root within interval [a, b].
  Function function;
  double a, b;   // Interval used for root finding.
  double guess;  // The initial guess. It must be in [a, b].
  double root;   // The true root in [a, b].
  double abs_tolerance{kEpsilon};  // Absolute tolerance for the solver.
  int max_iterations{100};  // Solver maximum number of iterations.
  // Expected number of iterations, if known.
  std::optional<int> num_iterations;
};

std::vector<RootFindingTestData> GenerateTestCases() {
  std::vector<RootFindingTestData> cases;
  cases.push_back({.description = "y = 1.5 * x + 3.0",
                   [](double x) {
                     double f = 1.5 * x + 3.0;
                     double dfx = 1.5;
                     return std::make_pair(f, dfx);
                   },
                   .a = -4.0,
                   .b = 3.0,
                   .guess = -0.5,
                   .root = -2.0,
                   .abs_tolerance = kEpsilon,
                   .max_iterations = 100,
                   // Since the function is linear, it reaches the solution in
                   // the first iteration (one evaluation).
                   // DoNewtonWithBisection() uses two additional evaluations to
                   // ensure the solution is bracketed.
                   .num_iterations = 3});

  // This function has two roots. We push them as two separate cases with two
  // different search intervals.
  cases.push_back({.description = "y = (x - 1.5) * (x + 2.0) = x² + 0.5 x − 3",
                   [](double x) {
                     double f = (x - 1.5) * (x + 2.0);
                     double dfx = 2.0 * x + 0.5;
                     return std::make_pair(f, dfx);
                   },
                   .a = -1.0, .b = 2.0, .guess = 1.0, .root = 1.5});
  cases.push_back({.description = "y = (x - 1.5) * (x + 2.0) = x² + 0.5 x − 3",
                   [](double x) {
                     double f = (x - 1.5) * (x + 2.0);
                     double dfx = 2.0 * x + 0.5;
                     return std::make_pair(f, dfx);
                   },
                   .a = -4.0, .b = 1.0, .guess = 1.0, .root = -2.0});

  cases.push_back({.description = "y = arctan(x)",
                   [](double x) {
                     double f = std::atan(x);
                     double dfx = 1.0 / (1.0 + x * x);
                     return std::make_pair(f, dfx);
                   },
                   .a = -1.0, .b = 10.0, .guess = 9.0, .root = 0.0});

  // For y = x³ − 2x + 2 with initial guess x0 = 0, Newton-Raphson will enter
  // into an infinity cycle without convergence.
  cases.push_back({.description = "y = x³ − 2x + 2",
                   [](double x) {
                     const double x2 = x * x;
                     const double f = x * (x2 - 2) + 2;
                     const double dfx = 3 * x2 - 2;
                     return std::make_pair(f, dfx);
                   },
                   .a = -3.0, .b = 3.0, .guess = 0.0,
                   .root = -1.769292354238631415240409464335});

  // This is a difficult case since y' = ∞ at the root x = 0.
  cases.push_back({.description = "y = sign(x) * abs(x)^(1/2)",
                   [](double x) {
                     const double s = x >= 0 ? 1.0 : -1.0;
                     const double sqrt_x = std::sqrt(std::abs(x));
                     const double f = s * sqrt_x;
                     const double dfx = 0.5 / sqrt_x;
                     return std::make_pair(f, dfx);
                   },
                   .a = -1.0, .b = 4.0, .guess = 2.0, .root = 0.0});

  // For y = x - tan(x) Newton-Raphson will diverge if the guess is outside
  // [4.3, 4.7].
  cases.push_back({.description = "y = x - tan(x)",
                   [](double x) {
                     const double tan_x = std::tan(x);
                     const double f = x - tan_x;
                     const double dfx = -tan_x * tan_x;
                     return std::make_pair(f, dfx);
                   },
                   .a = 2.0, .b = 4.7, .guess = 3.0,
                   .root = 4.4934094579090641753078809272803});

  // Newton-Raphson struggles with this case since the derivative also goes to
  // zero at the (triple) root x = 1.5.
  cases.push_back({.description = "y = (x-1.5)³",
                   [](double x) {
                     const double arg = x - 1.5;
                     const double f = arg * arg * arg;
                     const double dfx = 3.0 * arg * arg;
                     return std::make_pair(f, dfx);
                   },
                   .a = 0.0, .b = 2.0, .guess = 1.0, .root = 1.5});

  // Discontinuous derivative. A linear function plus a discontinuity will cause
  // the traditional Newton-Raphson to fall into an infinite cycle without
  // converging to the solution.
  cases.push_back({.description = "y = x + H(x+0.3)",
                   [](double x) {
                     const double f = x + ((x > -0.3) ? 1.0 : 0.0);
                     const double dfx = 1.0;
                     return std::make_pair(f, dfx);
                   },
                   .a = -1.0, .b = 1.5, .guess = 0.5, .root = -0.3});

  // This case is setup so that the derivative of the function at the initial
  // guess is zero, and therefore the Newton direction is not well defined.
  cases.push_back({.description = "y = x * (x - 1)",
                   [](double x) {
                     const double f = x * (x - 1.0);
                     const double dfx = 2.0 * x - 1.0;
                     return std::make_pair(f, dfx);
                   },
                   .a = -1.0, .b = 0.6, .guess = 0.5, .root = 0.0});

  // A discontinuous case. While there is no real root at x = 1.0, numerically
  // the solver sees the discontinuity as a sharp transition from negative to
  // positive within a narrow gap of size `abs_tolerance`.
  cases.push_back({.description = "y = 1.0 / (x - 1)",
                   [](double x) {
                     const double f = 1.0 / (x - 1.0);
                     const double dfx = - f * f;
                     return std::make_pair(f, dfx);
                   },
                   .a = -1.0, .b = 2.0, .guess = 0.0, .root = 1.0});

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

  // Find root in the interval [data.a, data.b]
  const auto [x, num_iterations] =
      DoNewtonWithBisectionFallback(data.function, data.a, data.b, data.guess,
                                    data.abs_tolerance, data.max_iterations);
  EXPECT_NEAR(x, data.root, data.abs_tolerance);
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
