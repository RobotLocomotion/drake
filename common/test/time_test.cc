#include "gtest/gtest.h"

#include "drake/common/timer.h"

namespace drake {
GTEST_TEST(TimeTest, TimerFunction) {
  std::unique_ptr<Timer> timer = std::make_unique<SteadyTimer>();
  timer->Start();
  // Assert that some positive time has passed
  EXPECT_GT(timer->Tick().count(), 0.0);
  EXPECT_GT(timer->Stop().count(), 0.0);
}

GTEST_TEST(TimeTest, ConstructorStartsTimer) {
  SteadyTimer timer{};
  // Assert that some positive time has passed
  EXPECT_GT(timer.Tick().count(), 0.0);
  EXPECT_GT(timer.Stop().count(), 0.0);
}
}