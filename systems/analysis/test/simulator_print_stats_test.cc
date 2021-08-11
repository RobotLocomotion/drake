#include "drake/systems/analysis/simulator_print_stats.h"

#include <gtest/gtest.h>

#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
namespace systems {
template <typename T>
class SimulatorPrintStatsTest : public::testing::Test {};

using MyTypes = ::testing::Types<double, AutoDiffXd>;
TYPED_TEST_SUITE(SimulatorPrintStatsTest, MyTypes);
TYPED_TEST(SimulatorPrintStatsTest, Test) {
  using T = TypeParam;
  // A no-crash test. Calling PrintSimulatorStatistics should not throw an
  // error.
  ConstantVectorSource<T> source(2);
  Simulator<T> simulator(source);
  simulator.AdvanceTo(2);

  PrintSimulatorStatistics(simulator);
}
}  // namespace systems
}  // namespace drake
