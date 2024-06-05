#include "drake/systems/analysis/realtime_rate_calculator.h"

#include <limits>

#include <gtest/gtest.h>

namespace drake {
namespace systems {
namespace {

using internal::RealtimeRateCalculator;

/* Mock timer allows the test to determine the time that Tick() reports. */
class ManualTimer final : public Timer {
 public:
  void Start() final { tick_ = 0.0; };
  double Tick() final { return tick_; }
  void set_tick(double tick) { tick_ = tick; }

 private:
  double tick_{0};
};

GTEST_TEST(RealtimeRateCalculatorTest, Calculation) {
  const double kPeriod = 1;
  RealtimeRateCalculator calculator(kPeriod);
  auto timer_ptr = std::make_unique<ManualTimer>();
  ManualTimer& timer = *timer_ptr;
  calculator.InjectMockTimer(std::move(timer_ptr));

  double sim_time = 1.0;
  timer.set_tick(kPeriod / 2);
  // Advancing less than the clock period produces no rate.
  auto rate_report = calculator.UpdateAndRecalculate(sim_time);
  EXPECT_EQ(rate_report.period_count, 0);
  // First invocation initializes the calculator.
  EXPECT_TRUE(rate_report.initialized);

  // Redundantly updating doesn't change the outcome.
  rate_report = calculator.UpdateAndRecalculate(sim_time);
  EXPECT_EQ(rate_report.period_count, 0);
  EXPECT_FALSE(rate_report.initialized);

  // Arbitrary advances to the simulation clock won't trigger a rate calculation
  // if time hasn't advanced enough.
  timer.set_tick(kPeriod * 0.75);
  sim_time += 3.0 * kPeriod;
  rate_report = calculator.UpdateAndRecalculate(sim_time);
  EXPECT_EQ(rate_report.period_count, 0);
  EXPECT_FALSE(rate_report.initialized);

  // Advancing to inside the next period *will* calculate a rate.
  timer.set_tick(kPeriod * 1.5);
  rate_report = calculator.UpdateAndRecalculate(sim_time);
  EXPECT_EQ(rate_report.period_count, 1);
  EXPECT_DOUBLE_EQ(rate_report.rate, sim_time / timer.Tick());
  EXPECT_FALSE(rate_report.initialized);

  // Advancing multiple periods will calculate a rate and indicate it applies to
  // multiple periods.

  // Calculating the expected rate will be tricky. The previous invocation
  // advanced simulation time to `sim_time` in 1.5 * kPeriod wall seconds. As
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
  // Having carefully conditioned the sim and wall time changes using the
  // expected residuals, we should be able to predict the reported rate.
  ASSERT_GT(sim_delta / wall_delta, 0);
  EXPECT_DOUBLE_EQ(rate_report.rate, sim_delta / wall_delta);

  // Another immediate call after an infinitesimal amount of time should
  // indicate that no rate can be computed.
  timer.set_tick(timer.Tick() + 1e-10);
  rate_report = calculator.UpdateAndRecalculate(sim_time);
  EXPECT_EQ(rate_report.period_count, 0);

  // We want to make sure the multi-period jump doesn't mess up the following
  // computation. So, we advance to *exactly* the next period boundary. We also
  // advance the simulation time. The total simulation time included in the rate
  // includes the residual from the previous calculation -- we were left with
  // a fraction of the previous simulation delta (0.1 / 4.1) that we have to
  // include in the calculation.
  timer.set_tick((std::floor(timer.Tick() / kPeriod) + 1) * kPeriod);
  rate_report = calculator.UpdateAndRecalculate(sim_time + sim_advance);
  EXPECT_EQ(rate_report.period_count, 1);
  EXPECT_DOUBLE_EQ(rate_report.rate,
                   (sim_advance + (0.1 / 4.1 * sim_delta)) / kPeriod);

  // Implicit calculator reset by moving backwards in time. No rate is reported.
  timer.set_tick(timer.Tick() + kPeriod * 1.5);
  rate_report = calculator.UpdateAndRecalculate(0.1);
  EXPECT_EQ(rate_report.period_count, 0);
  EXPECT_TRUE(rate_report.initialized);
  // Show that initialization called timer.Start().
  EXPECT_EQ(timer.Tick(), 0.0);

  // The previous invocation reset both the previous simulation time (to 0.1)
  // *and* the previous wall clock time (to timer.Tick()). So, even though
  // we've advanced wall time 2.1 periods since the last non-zero report, we've
  // only advanced 0.6 periods since the reset.
  timer.set_tick(timer.Tick() + kPeriod * 0.6);
  rate_report = calculator.UpdateAndRecalculate(1.1);
  EXPECT_EQ(rate_report.period_count, 0);
  EXPECT_FALSE(rate_report.initialized);

  // Advance to the next boundary based on the reset.
  timer.set_tick(timer.Tick() + kPeriod * 0.4);
  rate_report = calculator.UpdateAndRecalculate(2.1);
  EXPECT_EQ(rate_report.period_count, 1);
  EXPECT_DOUBLE_EQ(rate_report.rate, 2 / kPeriod);
  EXPECT_FALSE(rate_report.initialized);

  // Explicitly resetting it.
  calculator.Reset();
  rate_report = calculator.UpdateAndRecalculate(2.1);
  EXPECT_EQ(rate_report.period_count, 0);
  EXPECT_TRUE(rate_report.initialized);
}

// Confirm that setting the time to negative infinity "works" in the sense that
// -infinity counts as a time earlier than previous times for the purpose of
// re-initializing the calculator, but subsequent calls with finite, increasing
// times recalibrate to meaningful rate values.
GTEST_TEST(RealtimeRateCalculatorTest, NegativeInfinity) {
  const double kPeriod = 1;
  RealtimeRateCalculator calculator(kPeriod);
  auto timer_ptr = std::make_unique<ManualTimer>();
  ManualTimer& timer = *timer_ptr;
  calculator.InjectMockTimer(std::move(timer_ptr));

  // A reality check that we can advance time and get a rate.
  timer.set_tick(0);
  auto report = calculator.UpdateAndRecalculate(0);
  ASSERT_TRUE(report.initialized);
  // Advance wall clock and sim time in lock step; real time rate = 1.
  timer.set_tick(kPeriod);
  report = calculator.UpdateAndRecalculate(kPeriod);
  ASSERT_FALSE(report.initialized);
  EXPECT_DOUBLE_EQ(report.rate, 1.0);
  EXPECT_EQ(report.period_count, 1);

  // No set time to infinity; this should be an initialization event and a
  // subsequent elapsed period should produce an infinite rate.
  timer.set_tick(timer.Tick() + kPeriod);
  report =
      calculator.UpdateAndRecalculate(-std::numeric_limits<double>::infinity());
  ASSERT_TRUE(report.initialized);

  // After two elapsed periods, we should report two infinite rates.
  timer.set_tick(timer.Tick() + 2 * kPeriod);
  report = calculator.UpdateAndRecalculate(0);
  ASSERT_FALSE(report.initialized);
  EXPECT_DOUBLE_EQ(report.rate, std::numeric_limits<double>::infinity());
  EXPECT_EQ(report.period_count, 2);

  // After another period, we should be back to real rate measurements.
  timer.set_tick(timer.Tick() + kPeriod);
  report = calculator.UpdateAndRecalculate(kPeriod);
  ASSERT_FALSE(report.initialized);
  EXPECT_DOUBLE_EQ(report.rate, 1);
  EXPECT_EQ(report.period_count, 1);
}

}  // namespace
}  // namespace systems
}  // namespace drake
