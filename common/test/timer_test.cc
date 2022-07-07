#include "drake/common/timer.h"

#include <thread>

#include "gtest/gtest.h"

namespace drake {

GTEST_TEST(TimeTest, Everything) {
  SteadyTimer timer;

  // Clock starts upon construction. Allow some time to pass and confirm
  // positive measurement.
  EXPECT_GT(timer.Tick(), 0.0);

  // Start resets the clock. Allow a small time to pass and confirm timer
  // measures at least that amount.
  timer.Start();
  using std::chrono::duration;
  const duration<double> kShortInterval = std::chrono::milliseconds(10);
  std::this_thread::sleep_for(kShortInterval);  // sleep for at least 10ms
  const double t = timer.Tick();
  EXPECT_GT(t, 0.0);
  EXPECT_GE(t, std::chrono::duration<double>(kShortInterval).count());

  // As time progresses, the tick value monotonically increases.
  EXPECT_GT(timer.Tick(), t);
}
}  // namespace drake
