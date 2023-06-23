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
  auto timer = std::make_unique<MockTimer>();

  InstantaneousRealtimeRateCalculator calculator{};
  calculator.InjectMockTimer(std::move(timer));
  // First call is seed and gives nullopt result.
  EXPECT_FALSE(calculator.UpdateAndRecalculate(0.0).has_value());

  // Next calls return correct value.
  EXPECT_DOUBLE_EQ(calculator.UpdateAndRecalculate(0.1).value(), 1.0);
  EXPECT_DOUBLE_EQ(calculator.UpdateAndRecalculate(0.2).value(), 1.0);
  EXPECT_DOUBLE_EQ(calculator.UpdateAndRecalculate(0.4).value(), 2.0);

  // Reset time and observe another nullopt result.
  EXPECT_FALSE(calculator.UpdateAndRecalculate(0.0).has_value());
  EXPECT_DOUBLE_EQ(calculator.UpdateAndRecalculate(0.1).value(), 1.0);
}

// Tests that the InstantaneousRealtimeRateCalculator resets properly
// after simulation is paused.
GTEST_TEST(InstantaneousRealtimeRateCalculatorTest, UnpausedSimTest) {
  auto timer = std::make_unique<MockTimer>();

  InstantaneousRealtimeRateCalculator calculator{};
  calculator.InjectMockTimer(std::move(timer));
  // First call is seed and gives nullopt result.
  EXPECT_FALSE(calculator.UpdateAndRecalculate(0.0).has_value());

  // Next calls return correct value.
  EXPECT_DOUBLE_EQ(calculator.UpdateAndRecalculate(0.1).value(), 1.0);
  EXPECT_DOUBLE_EQ(calculator.UpdateAndRecalculate(0.2).value(), 1.0);
  EXPECT_DOUBLE_EQ(calculator.UpdateAndRecalculate(0.4).value(), 2.0);

  // Pause simulation for 2.0 seconds, then unpausing will reset the 
  // prev_sim_time_ to nullopt, so the next calculation will be correct.
  calculator.Reset();

  // Reset time and observe another nullopt result.
  EXPECT_FALSE(calculator.UpdateAndRecalculate(2.5).has_value());
  EXPECT_DOUBLE_EQ(calculator.UpdateAndRecalculate(2.6).value(), 1.0);
}
}  // namespace
}  // namespace systems
}  // namespace drake
