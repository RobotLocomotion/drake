#include "drake/examples/fibonacci/fibonacci_difference_equation.h"

#include <gtest/gtest.h>

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/vector_log_sink.h"

namespace drake {
namespace examples {
namespace fibonacci {
namespace {

// Verify that we get the right sequence for one sequence length.
GTEST_TEST(Fibonacci, CheckSequence) {
  systems::DiagramBuilder<double> builder;
  auto fibonacci = builder.AddSystem<FibonacciDifferenceEquation>();
  auto logger = builder.AddSystem<systems::VectorLogSink<double>>(
      1,  // Size of input.
      FibonacciDifferenceEquation::kPeriod);
  builder.Connect(fibonacci->GetOutputPort("Fn"), logger->GetInputPort("data"));
  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  const auto& log = logger->FindLog(simulator.get_context());

  // Simulate forward to fibonacci(6): 0 1 1 2 3 5 8
  simulator.AdvanceTo(6 * FibonacciDifferenceEquation::kPeriod);

  Eigen::VectorXd expected(7);
  expected << 0, 1, 1, 2, 3, 5, 8;

  EXPECT_EQ(log.data().transpose(), expected);
}

}  // namespace
}  // namespace fibonacci
}  // namespace examples
}  // namespace drake
