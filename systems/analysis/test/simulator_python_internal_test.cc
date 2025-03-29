#include "drake/systems/analysis/simulator_python_internal.h"

#include <gtest/gtest.h>

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
namespace systems {
namespace internal {
namespace {

// The python_monitor will increment this mutable, global counter.
int global_counter = 0;

GTEST_TEST(SimulatorPythonInternalTest, BasicTest) {
  ConstantVectorSource<double> source(1.0);
  Simulator<double> simulator(source);

  // Add a monitor. Initializing or advancing should update the counter.
  global_counter = 0;
  auto python_monitor = []() {
    ++global_counter;
  };
  SimulatorPythonInternal<double>::set_python_monitor(&simulator,
                                                      python_monitor);
  simulator.Initialize();
  EXPECT_EQ(global_counter, 1);
  simulator.AdvanceTo(0.2);
  EXPECT_EQ(global_counter, 2);
  simulator.AdvanceTo(0.3);
  EXPECT_EQ(global_counter, 3);

  // Clearing the monitor means no more counter updates.
  SimulatorPythonInternal<double>::set_python_monitor(&simulator, nullptr);
  simulator.Initialize();
  simulator.AdvanceTo(0.4);
  EXPECT_EQ(global_counter, 3);
}

}  // namespace
}  // namespace internal
}  // namespace systems
}  // namespace drake
