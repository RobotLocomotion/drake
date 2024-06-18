#include "drake/systems/analysis/realtime_rate_calculator.h"

#include <gtest/gtest.h>

namespace drake {
namespace systems {
namespace {

using internal::RealtimeRateCalculator;
using internal::PeriodicRealtimeRateCalculator;

class MockTimer final : public Timer {
 public:
  void Start() final {}
  double Tick() final { return 0.1; }
};

/* Mock timer allows me to determine the time that Tick() reports. */
class ManualTimer final : public Timer {
 public:
  void Start() final {};
  double Tick() final { return tick_; }
  void set_tick(double tick) { tick_ = tick; }
 private:
  double tick_{0};
};

// Tests that the RealtimeRateCalculator calculates the correct
// results including startup and rewinding time.
GTEST_TEST(RealtimeRateCalculatorTest, BasicTest) {
  RealtimeRateCalculator calculator{};
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

  // Calling UpdateAndRecalcuate() again with the same sim time will
  // return 0.0 realtime rate.
  EXPECT_DOUBLE_EQ(calculator.UpdateAndRecalculate(0.1).value(), 0.0);

  // RealtimeRateCalculator should reset properly on call to Reset().
  calculator.Reset();

  // First call to UpdateAndRecalculate() after Reset() returns nullopt
  // and calculation resumes on the next call.
  EXPECT_FALSE(calculator.UpdateAndRecalculate(0.2).has_value());
  EXPECT_DOUBLE_EQ(calculator.UpdateAndRecalculate(0.3).value(), 1.0);
}

GTEST_TEST(PeriodicRealtimeRateCalculatorTest, PeriodInSimSeconds) {
  const double kPeriod = 0.25;
  PeriodicRealtimeRateCalculator calculator(kPeriod,
                                            /* period_in_sim_time */ true);
  auto timer_ptr = std::make_unique<ManualTimer>();
  ManualTimer& timer = *timer_ptr;
  calculator.InjectMockTimer(std::move(timer_ptr));

  double sim_time = kPeriod / 2;
  // Advancing less than the simulation clock period produces no rate.
  auto rate_report = calculator.UpdateAndRecalculate(sim_time);
  EXPECT_EQ(rate_report.period_count, 0);

  // Redundantly updating doesn't change the outcome.
  rate_report = calculator.UpdateAndRecalculate(sim_time);
  EXPECT_EQ(rate_report.period_count, 0);

  // Arbitrary advances to the wall clock won't trigger a rate calculation.
  timer.set_tick(3.5 * kPeriod);
  rate_report = calculator.UpdateAndRecalculate(sim_time);
  EXPECT_EQ(rate_report.period_count, 0);

  // Advancing to inside the next period *will* calculate a rate.
  sim_time = kPeriod * 1.5;
  rate_report = calculator.UpdateAndRecalculate(sim_time);
  EXPECT_EQ(rate_report.period_count, 1);
  EXPECT_DOUBLE_EQ(rate_report.rate, sim_time / timer.Tick());

  // Advancing multiple periods will calculate a rate and indicate more pending
  // reports.

  // Calculating the expected rate will be tricky. The previous invocation
  // advanced simulation time by 1.5 periods in timer.Tick() wall seconds. As
  // part of the calculation, we clip the simulation and wall times to that
  // portion within the full boundary, leaving a pair of residual times. 1/3
  // of the sim time and wall time are residual. As we consider what the change
  // in wall clock time, we include the residual (as well as advancing the
  // wall clock).
  const double wall_advance = 2.0;
  // Sim time at previous calculation was at 1.5 periods. After clipping to the
  // period boundary, the residual time is 1/3 of the wall time.
  const double residual_wall_time = timer.Tick() / 3;
  const double wall_delta = residual_wall_time + wall_advance;
  timer.set_tick(timer.Tick() + wall_advance);
  // We'll advance the simulation time 3.6 periods. However, we have residual
  // sim time of 0.5 periods. The full elapsed sim time will be the sum of these
  // two values (producing an extra period report).
  const double sim_advance = kPeriod * 3.6;
  sim_time += sim_advance;
  double sim_delta = sim_advance + kPeriod * 0.5;  // 4.1 periods.
  rate_report = calculator.UpdateAndRecalculate(sim_time);
  EXPECT_EQ(rate_report.period_count, 4);
  // Haven't carefully conditioned the sim and wall time changes using the
  // expected residuals, we should be able to predict the reported rate.
  EXPECT_DOUBLE_EQ(rate_report.rate, sim_delta / wall_delta);

  // Now we can pump the calculator to get multiple periods of uniform rate.
  for (int i = 1; i < 4; ++i) {
    rate_report = calculator.UpdateAndRecalculate(sim_time);
    EXPECT_EQ(rate_report.period_count, 4 - i);
    EXPECT_DOUBLE_EQ(rate_report.rate, sim_delta / wall_delta);
  }
  // One more call, will return no value.
  rate_report = calculator.UpdateAndRecalculate(sim_time);
  EXPECT_EQ(rate_report.period_count, 0);
}

GTEST_TEST(PeriodicRealtimeRateCalculatorTest, PeriodInWallSeconds) {
  const double kPeriod = 1;  // 0.25;
  PeriodicRealtimeRateCalculator calculator(kPeriod,
                                            /* period_in_sim_time */ false);
  auto timer_ptr = std::make_unique<ManualTimer>();
  ManualTimer& timer = *timer_ptr;
  calculator.InjectMockTimer(std::move(timer_ptr));

  double sim_time = 1.0;
  timer.set_tick(kPeriod / 2);
  // Advancing less than the clock period produces no rate.
  auto rate_report = calculator.UpdateAndRecalculate(sim_time);
  EXPECT_EQ(rate_report.period_count, 0);

  // Redundantly updating doesn't change the outcome.
  rate_report = calculator.UpdateAndRecalculate(sim_time);
  EXPECT_EQ(rate_report.period_count, 0);

  // Arbitrary advances to the simulation clock won't trigger a rate
  // calculation.
  sim_time += 3.0 * kPeriod;
  rate_report = calculator.UpdateAndRecalculate(sim_time);
  EXPECT_EQ(rate_report.period_count, 0);

  // Advancing to inside the next period *will* calculate a rate.
  timer.set_tick(kPeriod * 1.5);
  rate_report = calculator.UpdateAndRecalculate(sim_time);
  EXPECT_EQ(rate_report.period_count, 1);
  EXPECT_DOUBLE_EQ(rate_report.rate, sim_time / timer.Tick());

  // Advancing multiple periods will calculate a rate and indicate more pending
  // reports.

  // Calculating the expected rate will be tricky. The previous invocation
  // advanced simulation time to sim_time in 1.5 * kPeriod wall seconds. As
  // part of the calculation, we clip the simulation and wall times to that
  // portion within the full boundary, leaving a pair of residual times. 1/3
  // of the sim time and wall time are residual. As we advance the timer and
  // consider what the "change in wall clock" time used to compute rate, we
  // need to include the advancement and residual.
  //
  // Note: we're *advancing* the wall clock by 3.6 periods. However, the
  // residual wall time of 0.5 periods contribute in determining elapsed time.
  // So, rather than reporting available for three full periods, we'll get four.
  const double wall_advance = kPeriod * 3.6;
  const double residual_wall_time = kPeriod * 0.5;
  const double wall_delta = residual_wall_time + wall_advance;  // 4.1 periods.
  timer.set_tick(timer.Tick() + wall_advance);
  // We can advance sim_time an arbitrary amount, but the total elapsed sim time
  // will include the 1/3 of the elapsed time.
  const double sim_advance = 2.0;
  const double sim_residual = sim_time / 3;
  sim_time += sim_advance;
  double sim_delta = sim_advance + sim_residual;
  rate_report = calculator.UpdateAndRecalculate(sim_time);
  EXPECT_EQ(rate_report.period_count, 4);
  // Haven't carefully conditioned the sim and wall time changes using the
  // expected residuals, we should be able to predict the reported rate.
  EXPECT_DOUBLE_EQ(rate_report.rate, sim_delta / wall_delta);

  // Now we can pump the calculator to get multiple periods of uniform rate.
  for (int i = 1; i < 4; ++i) {
    rate_report = calculator.UpdateAndRecalculate(sim_time);
    EXPECT_EQ(rate_report.period_count, 4 - i);
    EXPECT_DOUBLE_EQ(rate_report.rate, sim_delta / wall_delta);
  }
  // One more call, will return no value.
  rate_report = calculator.UpdateAndRecalculate(sim_time);
  EXPECT_EQ(rate_report.period_count, 0);

  // There is an outlier case where the wall clock has advanced enough to
  // justify a report, but the sim_time passed produces a zero or negative
  // amount of elapsed sim time.
  timer.set_tick(timer.Tick() + kPeriod);
  EXPECT_EQ(calculator.UpdateAndRecalculate(0.1).period_count, 0);
}

}  // namespace
}  // namespace systems
}  // namespace drake
