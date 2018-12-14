#include "drake/examples/fibonacci/fibonacci_difference_equation.h"

#include <gtest/gtest.h>

#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace examples {
namespace fibonacci {
namespace {

// Verify that we get the right sequence for one sequence length.
GTEST_TEST(Fibonacci, CheckSequence) {
  FibonacciDifferenceEquation fibonacci;

  systems::Simulator<double> simulator(fibonacci);

  // Simulate forward to fibonacci(6): 0 1 1 2 3 5 8
  testing::internal::CaptureStdout();
  simulator.StepTo(6 * FibonacciDifferenceEquation::kPeriod);
  std::string output = testing::internal::GetCapturedStdout();

  EXPECT_EQ(output,
            "0: 0\n"
            "1: 1\n"
            "2: 1\n"
            "3: 2\n"
            "4: 3\n"
            "5: 5\n"
            "6: 8\n");
}

}  // namespace
}  // namespace fibonacci
}  // namespace examples
}  // namespace drake
