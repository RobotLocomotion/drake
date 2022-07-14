#include "drake/common/timer.h"

#include <thread>

#include "gtest/gtest.h"

namespace drake {

GTEST_TEST(TimeTest, Everything) {
  SteadyTimer timer;

  // Clock starts upon construction. Allow some time to pass and confirm
  // positive measurement.
  using std::chrono::duration;
  const duration<double> kTestInterval = std::chrono::milliseconds(100);
  std::this_thread::sleep_for(kTestInterval);  // sleep for at least 100ms
  const double T1 = timer.Tick();
  EXPECT_GT(T1, 0.0);
  EXPECT_GE(T1, kTestInterval.count());

  // Start restarts the timer, the new measurement should be less than T1.
  timer.Start();
  const double T2 = timer.Tick();
  EXPECT_GE(T2, 0.0);
  EXPECT_LT(T2, T1);  // ensure we measured less time after call to Start()

  // As time progresses, the tick value monotonically increases.
  EXPECT_GT(timer.Tick(), T2);
}
}  // namespace drake
