#include <gflags/gflags.h>

#include "drake/examples/fibonacci/fibonacci.h"
#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace examples {
namespace fibonacci {
namespace {

DEFINE_int32(steps, 10, "Length of Fibonacci sequence to generate.");

// Use Drake's hybrid Simulator to produce the Fibonacci sequence up to
// the step number supplied on the command line (default 10).
int DoMain() {
  FibonacciDifferenceEquation fibonacci;

  systems::Simulator<double> simulator(fibonacci);

  // Set the initial conditions.
  fibonacci.Initialize(&simulator.get_mutable_context());

  // Simulate forward until t=h*steps.
  simulator.StepTo(FibonacciDifferenceEquation::kPeriod * FLAGS_steps);

  return 0;
}

}  // namespace
}  // namespace fibonacci
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage("usage: run_fibonacci [--steps=n]");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::fibonacci::DoMain();
}
