#include "drake/systems/analysis/instantaneous_realtime_rate_calculator.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace drake {
namespace systems {
namespace {

using ::testing::Return;

class MockTimer : public Timer {
 public:
  MOCK_METHOD(Timer::duration, Tick, (), (override));
  MOCK_METHOD(void, Start, (), (override));
};

// Tests that the InstantaneousRealtimeRateCalculator calculates the correct
// results including startup and rewinding time
GTEST_TEST(InstantaneousRealtimeRateCalculatorTest, BasicTest) {
  auto timer = std::make_unique<MockTimer>();
  const double kSleepSec = 0.1;
  EXPECT_CALL(*timer, Tick)
      .WillRepeatedly(Return(MockTimer::duration(kSleepSec)));

  InstantaneousRealtimeRateCalculator calculator{};
  calculator.InjectMockTimer(std::move(timer));
  // first call is seed, no result
  EXPECT_FALSE(calculator.CalculateRealtimeRate(0.0).has_value());

  // next calls return correct value
  EXPECT_DOUBLE_EQ(calculator.CalculateRealtimeRate(0.1).value(), 1.0);
  EXPECT_DOUBLE_EQ(calculator.CalculateRealtimeRate(0.2).value(), 1.0);
  EXPECT_DOUBLE_EQ(calculator.CalculateRealtimeRate(0.4).value(), 2.0);

  // reset time
  EXPECT_FALSE(calculator.CalculateRealtimeRate(0.0).has_value());
  EXPECT_DOUBLE_EQ(calculator.CalculateRealtimeRate(0.1).value(), 1.0);
}
}  // namespace
}  // namespace systems
}  // namespace drake
