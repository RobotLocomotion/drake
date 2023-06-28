#include "drake/systems/analysis/instantaneous_realtime_rate_calculator.h"

#include <gtest/gtest.h>

namespace drake {
namespace systems {
namespace {


using internal::InstantaneousRealtimeRateCalculator;

class MockTimer final : public Timer {
 public:
  void Start() final {}
  double Tick() final { return 0.1; }
};

// Tests that the InstantaneousRealtimeRateCalculator calculates the correct
// results including startup and rewinding time.
GTEST_TEST(InstantaneousRealtimeRateCalculatorTest, BasicTest) {
  InstantaneousRealtimeRateCalculator calculator{};
  calculator.InjectMockTimer(std::make_unique<MockTimer>());
  // First (seed) call returns nullopt.
  EXPECT_FALSE(calculator.UpdateAndRecalculate(0.0).has_value());

  // Next calls return correct value.
  EXPECT_DOUBLE_EQ(calculator.UpdateAndRecalculate(0.1).value(), 1.0);
  EXPECT_DOUBLE_EQ(calculator.UpdateAndRecalculate(0.2).value(), 1.0);
  EXPECT_DOUBLE_EQ(calculator.UpdateAndRecalculate(0.4).value(), 2.0);

  // Reset time and observe another nullopt result.
  EXPECT_FALSE(calculator.UpdateAndRecalculate(0.0).has_value());
  EXPECT_DOUBLE_EQ(calculator.UpdateAndRecalculate(0.1).value(), 1.0);

  // Pause simulator and RTR should be zero. Note that in practice meshcat
  // still shows the RTR before pause since the simulation is paused and
  // meshcat system is not publishing updated RTRs anymore.
  EXPECT_DOUBLE_EQ(calculator.UpdateAndRecalculate(0.1).value(), 0.0);

  calculator.Reset();

  // First call to UpdateAndRecalculate() after Reset() returns nullopt
  // and calculation resumes on the next call.
  EXPECT_FALSE(calculator.UpdateAndRecalculate(0.2).has_value());
  EXPECT_DOUBLE_EQ(calculator.UpdateAndRecalculate(0.3).value(), 1.0);
}
}  // namespace
}  // namespace systems
}  // namespace drake
