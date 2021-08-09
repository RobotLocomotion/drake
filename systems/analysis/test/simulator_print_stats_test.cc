#include "drake/systems/analysis/simulator_print_stats.h"

#include <gtest/gtest.h>

#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
namespace systems {
template <typename T>
void PrintSimulatorStatisticsTester() {
  // A no-crash test. Calling PrintSimulatorStatistics should not throw an
  // error.
  ConstantVectorSource<T> source(2);
  Simulator<T> simulator(source);
  simulator.AdvanceTo(2);

  PrintSimulatorStatistics(simulator);
}

GTEST_TEST(PrintSimulatorStatisticsTest, Test) {
  PrintSimulatorStatisticsTester<double>();
  PrintSimulatorStatisticsTester<AutoDiffXd>();
}
}  // namespace systems
}  // namespace drake
