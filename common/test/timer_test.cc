#include "drake/common/timer.h"

#include <thread>

#include "gtest/gtest.h"

namespace drake {

GTEST_TEST(TimeTest, Everything) {
  using std::chrono::duration;
  const duration<double> kLongInterval = std::chrono::milliseconds(100);
  const duration<double> kShortInterval = std::chrono::milliseconds(10);

  SteadyTimer timer;

  // Clock starts upon construction. Allow some time to pass and confirm
  // positive measurement.
  std::this_thread::sleep_for(kLongInterval);
  const double T1 = timer.Tick();
  EXPECT_GT(T1, 0.0);

  // Start resets the clock. Allow a smaller time to pass and confirm positive
  // measurement that is less than T1.
  timer.Start();
  std::this_thread::sleep_for(kShortInterval);
  const double T2 = timer.Tick();
  EXPECT_GT(T2, 0.0);
  EXPECT_LT(T2, T1);

  // As time progresses, the tick value monotonically increases.
  EXPECT_GT(timer.Tick(), T2);
}
}  // namespace drake
