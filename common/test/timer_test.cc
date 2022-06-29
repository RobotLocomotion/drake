#include "drake/common/timer.h"

#include "gtest/gtest.h"

namespace drake {
GTEST_TEST(TimeTest, TimerFunction) {
  std::unique_ptr<Timer> timer = std::make_unique<SteadyTimer>();
  timer->Start();
  // Assert that some positive time has passed.
  const auto T1 = timer->Tick();
  EXPECT_GT(T1, 0.0);

  // Assert time advances.
  const auto T2 = timer->Tick();
  EXPECT_GT(T2, T1);
}

GTEST_TEST(TimeTest, ConstructorStartsTimer) {
  SteadyTimer timer{};
  // Assert that some positive time has passed.
  EXPECT_GT(timer.Tick(), 0.0);
}
}  // namespace drake
