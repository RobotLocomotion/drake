#include <gflags/gflags.h>

#include "drake/examples/fibonacci/fibonacci_difference_equation.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/vector_log_sink.h"

DEFINE_int32(steps, 10, "Length of Fibonacci sequence to generate.");

namespace drake {
namespace examples {
namespace fibonacci {
namespace {

// Use Drake's hybrid Simulator to produce the Fibonacci sequence up to
// the step number supplied on the command line (default 10).
int main(int argc, char* argv[]) {
  // Handle the command line "steps" argument.
  gflags::SetUsageMessage("usage: run_fibonacci [--steps=n]");
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Build a Diagram containing the Fibonacci system and a data logger that
  // samples the Fibonacci output port exactly at the update times.
  systems::DiagramBuilder<double> builder;
  auto fibonacci = builder.AddSystem<FibonacciDifferenceEquation>();
  auto logger = LogVectorOutput(fibonacci->GetOutputPort("Fn"), &builder,
                                FibonacciDifferenceEquation::kPeriod);
  auto diagram = builder.Build();

  // Create a Simulator and use it to advance time until t=steps*h.
  systems::Simulator<double> simulator(*diagram);
  simulator.AdvanceTo(FLAGS_steps * FibonacciDifferenceEquation::kPeriod);

  // Print out the contents of the log.
  const auto& log = logger->FindLog(simulator.get_context());
  for (int n = 0; n < log.sample_times().size(); ++n) {
    const double t = log.sample_times()[n];
    std::cout << n << ": " << log.data()(0, n)
              << " (t=" << t << ")\n";
  }

  return 0;
}

}  // namespace
}  // namespace fibonacci
}  // namespace examples
}  // namespace drake

int main(int argc, char **argv) {
  return drake::examples::fibonacci::main(argc, argv);
}
